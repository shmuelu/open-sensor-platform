/*
 * ami306.c is driver for AMI306 (Aichi Steel) and
 * AK8974 (Asahi Kasei EMD Corporation) magnetometer chip
 *
 * Copyright (C) 2009 Nokia Corporation and/or its subsidiary(-ies).
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/i2c/ami306.h>
#include <linux/input-polldev.h>
#include <linux/slab.h>

#define DRV_NAME "ami306"

/*
 * 16-bit registers are little-endian. LSB is at the address defined below
 * and MSB is at the next higher address.
 */
#define AMI306_SELFTEST		0x0C
#define AMI306_INFO		0x0D
#define AMI306_WHOAMI		0x0F
#define AMI306_DATA_X		0x10
#define AMI306_DATA_Y		0x12
#define AMI306_DATA_Z		0x14
#define AMI306_INT_SRC		0x16
#define AMI306_STATUS		0x18
#define AMI306_INT_CLEAR	0x1A
#define AMI306_CTRL1		0x1B
#define AMI306_CTRL2		0x1C
#define AMI306_CTRL3		0x1D
#define AMI306_INT_CTRL		0x1E
#define AMI306_OFFSET_X		0x20
#define AMI306_OFFSET_Y		0x22
#define AMI306_OFFSET_Z		0x24
#define AMI306_INT_THRES	0x26  /* absolute any axis value threshold */
#define AMI306_PRESET		0x30
#define AMI306_TEMP		0x31

#define AMI306_SELFTEST_IDLE	0x55
#define AMI306_SELFTEST_OK	0xAA

#define AMI306_WHOAMI_VALUE_AMI307 0xff
#define AMI306_WHOAMI_VALUE_AMI305 0x46
#define AMI306_WHOAMI_VALUE_AMI306 0x47
#define AMI306_WHOAMI_VALUE_AK8974 0x48

#define AMI306_INT_X_HIGH	0x80  /* Axis over +threshold  */
#define AMI306_INT_Y_HIGH	0x40
#define AMI306_INT_Z_HIGH	0x20
#define AMI306_INT_X_LOW	0x10  /* Axis below -threshold	*/
#define AMI306_INT_Y_LOW	0x08
#define AMI306_INT_Z_LOW	0x04
#define AMI306_INT_RANGE	0x02  /* Range overflow (any axis)   */

#define AMI306_STATUS_DRDY	0x40  /* Data ready	    */
#define AMI306_STATUS_OVERRUN	0x20  /* Data overrun	    */
#define AMI306_STATUS_INT	0x10  /* Interrupt occurred */

#define AMI306_CTRL1_POWER	0x80  /* 0 = standby; 1 = active */
#define AMI306_CTRL1_RATE	0x10  /* 0 = 10 Hz;   1 = 20 Hz	 */
#define AMI306_CTRL1_FORCE_EN	0x02  /* 0 = normal;  1 = force	 */
#define AMI306_CTRL1_MODE2	0x01  /* 0 */

#define AMI306_CTRL2_INT_EN	0x10  /* 1 = enable interrupts	      */
#define AMI306_CTRL2_DRDY_EN	0x08  /* 1 = enable data ready signal */
#define AMI306_CTRL2_DRDY_POL	0x04  /* 1 = data ready active high   */

#define AMI306_CTRL3_RESET	0x80  /* Software reset		  */
#define AMI306_CTRL3_FORCE	0x40  /* Start forced measurement */
#define AMI306_CTRL3_SELFTEST	0x10  /* Set selftest register	  */

#define AMI306_INT_CTRL_XEN	0x80  /* Enable interrupt for this axis */
#define AMI306_INT_CTRL_YEN	0x40
#define AMI306_INT_CTRL_ZEN	0x20
#define AMI306_INT_CTRL_XYZEN	0xE0
#define AMI306_INT_CTRL_POL	0x08  /* 0 = active low; 1 = active high     */
#define AMI306_INT_CTRL_PULSE	0x02  /* 0 = latched;	 1 = pulse (50 usec) */

#define AMI306_MAX_RANGE	2048
#define AMI306_THRESHOLD_MAX	(AMI306_MAX_RANGE - 1)

#define AMI306_POLL_INTERVAL	20   	/* ms */
#define AMI306_ACTIVATE_DELAY	1     /* ms */
#define AMI306_DEFAULT_FUZZ	3     /* input noise filtering */
#define AMI306_MEAS_DELAY	6     /* one measurement in ms */
#define AMI306_SELFTEST_DELAY	1     /* ms. Spec says 200us min */
#define AMI306_RESET_DELAY	5     /* ms */

#define AMI306_PWR_ON		1
#define AMI306_PWR_OFF		0

#define AMI306_MAX_TRY		2

struct ami306_chip {
	struct mutex		lock;	/* Serialize access to chip */
	struct mutex		users_lock;
	struct i2c_client	*client;
	struct input_polled_dev *idev;	   /* input device */

	int			max_range;
	int			users;
	int			fuzz;

	s16			x, y, z; /* Latest measurements */
};

static int ami306_write(struct ami306_chip *chip, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(chip->client, reg, data);
}

static int ami306_read(struct ami306_chip *chip, u8 reg)
{
	return i2c_smbus_read_byte_data(chip->client, reg);
}

static int ami306_read_block(struct ami306_chip *chip, u8 reg,
			     u8 *data, u8 length)
{
	s32 result;
	result = i2c_smbus_read_i2c_block_data(chip->client,
				reg, length, data);
	return result;
}

static int ami306_power(struct ami306_chip *chip, int poweron)
{
	int r, v;

	v = poweron ? AMI306_CTRL1_POWER : 0;
	v = v | AMI306_CTRL1_FORCE_EN;
	r = ami306_write(chip, AMI306_CTRL1, v);
	msleep(5);
	if (r < 0)
		return r;

	msleep(5);
	if (poweron)
		msleep(AMI306_ACTIVATE_DELAY);

	return 0;
}

static int ami306_start_measurement(struct ami306_chip *chip)
{
	int ctrl3;
	int ret = 0;

	ctrl3 = ami306_read(chip, AMI306_CTRL3);
	if (ctrl3 < 0)
		return ctrl3;

	ret = ami306_write(chip, AMI306_CTRL3, ctrl3 | AMI306_CTRL3_FORCE);

	return ret;
}

static int ami306_reset(struct ami306_chip *chip)
{
	int r = 0;

	/* Power on to get register access */
	r = ami306_power(chip, AMI306_PWR_ON);
	if (r < 0)
		goto fail;

	r = ami306_write(chip, AMI306_CTRL3, AMI306_CTRL3_RESET);
	if (r < 0)
		goto fail;

	msleep(AMI306_RESET_DELAY);
fail:
	return r;
}

static int ami306_add_users(struct ami306_chip *chip)
{
	int r = 0;

	mutex_lock(&chip->users_lock);

	if (chip->users == 0) {
		r = ami306_power(chip, AMI306_PWR_ON);
		if (r < 0)
			goto fail;
	}
	chip->users++;
fail:
	mutex_unlock(&chip->users_lock);
	return r;
}

static int ami306_remove_users(struct ami306_chip *chip)
{
	int r = 0;

	mutex_lock(&chip->users_lock);

	if (chip->users != 0)
		chip->users--;

	if (chip->users == 0) {
		r = ami306_power(chip, AMI306_PWR_OFF);
		if (r < 0)
			goto fail;
	}
fail:
	mutex_unlock(&chip->users_lock);
	return r;
}

static int ami306_configure(struct ami306_chip *chip)
{
	int err;

	ami306_reset(chip);

	ami306_add_users(chip);

	err = ami306_write(chip, AMI306_CTRL2, AMI306_CTRL2_DRDY_EN);
	if (err)
		goto fail;

	err = ami306_write(chip, AMI306_CTRL3, 0);
	if (err)
		goto fail;

	err = ami306_write(chip, AMI306_INT_CTRL, AMI306_INT_CTRL_POL);
	if (err)
		goto fail;

	err = ami306_write(chip, AMI306_PRESET, 0);
	if (err)
		goto fail;

fail:
	ami306_remove_users(chip);

	return err;
}


/*
 * Reads the value from the last TM command. 
 * Expects the TM command to be done at least once by the init
 *   or from the last iteration of this loop.
 */
static int ami306_read_values(struct ami306_chip *chip)
{
	s16 hw_values[3];
	int i;
	int ret = 0;
	static int first = 1;

	if (!first) {
		i = AMI306_MAX_TRY;
		do {
			if (ami306_read(chip, AMI306_STATUS) & AMI306_STATUS_DRDY)
				break;
			msleep(AMI306_MEAS_DELAY);
			i--;
		} while (i > 0);

		if (i == 0) {
			dev_err(&chip->client->dev, "Chip never reported ready.\n");
			first = 1;
			return -ENODEV;
		}

		/* X, Y, Z are in conscutive addresses. 2 * 3 bytes */
		ret = ami306_read_block(chip, AMI306_DATA_X, (u8 *)hw_values, 6);
		if (ret != 6) {
			dev_err(&chip->client->dev, "Read measurement failure. (%i)\n", ret);
		}

		for (i = 0; i < 3; i++)
			hw_values[i] = le16_to_cpu(hw_values[i]);

		chip->x = hw_values[0];
		chip->y = hw_values[1];
		chip->z = hw_values[2];
	}

	first = 0;

	ret = ami306_start_measurement(chip);
	if (ret != 0) {
		dev_err(&chip->client->dev, "Start measurement failure. (%i)\n", ret);
	}

	return 0;
}

/*
 * Self test is not defined in AMI307/AMI306 datasheets.
 */
static int ami306_selftest(struct ami306_chip *chip)
{
	int r;
	int success = 0;

	ami306_add_users(chip);

	r = ami306_read(chip, AMI306_SELFTEST);
	if (r != AMI306_SELFTEST_IDLE)
		goto out;

	r = ami306_read(chip, AMI306_CTRL3);
	if (r < 0)
		goto out;

	r = ami306_write(chip, AMI306_CTRL3, r | AMI306_CTRL3_SELFTEST);
	if (r < 0)
		goto out;

	msleep(AMI306_SELFTEST_DELAY);

	r = ami306_read(chip, AMI306_SELFTEST);
	if (r != AMI306_SELFTEST_OK)
		goto out;

	r = ami306_read(chip, AMI306_SELFTEST);
	if (r == AMI306_SELFTEST_IDLE)
		success = 1;

 out:
	ami306_remove_users(chip);

	return success;
}

static void ami306_set_input_params(struct ami306_chip *chip, int fuzz)
{
	struct input_dev *input_dev = chip->idev->input;
	int range;

	range = chip->max_range;
	input_set_abs_params(input_dev, ABS_X, -range, range, fuzz, 0);
	input_set_abs_params(input_dev, ABS_Y, -range, range, fuzz, 0);
	input_set_abs_params(input_dev, ABS_Z, -range, range, fuzz, 0);
	chip->fuzz = fuzz;
}
/*
 * SYSFS interface
 */

static ssize_t ami306_show_selftest(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ami306_chip *chip = dev_get_drvdata(dev);
	char *status;

	mutex_lock(&chip->lock);
	status = ami306_selftest(chip) ? "OK" : "FAIL";
	mutex_unlock(&chip->lock);

	return sprintf(buf, "%s\n", status);
}

static DEVICE_ATTR(selftest, S_IRUGO, ami306_show_selftest, NULL);

static ssize_t ami306_show_active(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ami306_chip *chip = dev_get_drvdata(dev);
	int status;

	/* Read from the chip to reflect real state of the HW */
	status = ami306_read(chip, AMI306_CTRL1) & AMI306_CTRL1_POWER;
	return sprintf(buf, "%s\n", status ? "ON" : "OFF");
}

static DEVICE_ATTR(active, S_IRUGO, ami306_show_active, NULL);

static ssize_t ami306_get_fuzz(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ami306_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", chip->fuzz);
}

static ssize_t ami306_set_fuzz(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct ami306_chip *chip = dev_get_drvdata(dev);
	unsigned long fuzz;

	if (strict_strtoul(buf, 0, &fuzz))
		return -EINVAL;

	if (fuzz > AMI306_MAX_RANGE)
		return -EINVAL;

	ami306_set_input_params(chip, fuzz);

	return count;
}

static DEVICE_ATTR(fuzz, S_IRUGO | S_IWUSR, ami306_get_fuzz,
					    ami306_set_fuzz);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_selftest.attr,
	&dev_attr_active.attr,
	&dev_attr_fuzz.attr,
	NULL
};

static struct attribute_group ami306_attribute_group = {
	.attrs = sysfs_attrs
};
 
/*
 * Polled input device interface
 */

static void ami306_poll(struct input_polled_dev *pidev)
{
	struct ami306_chip *chip = pidev->private;

	mutex_lock(&chip->lock);

	ami306_read_values(chip);
#if 0
	printk("HY-DBG: %s:%i x = 0x%02x, y = 0x%02x, z = 0x%02x\n", __func__, __LINE__,
		chip->x, chip->y, chip->z);
#endif
	input_report_abs(pidev->input, ABS_X, chip->x);
	input_report_abs(pidev->input, ABS_Y, -(chip->y));
	input_report_abs(pidev->input, ABS_Z, -(chip->z));
	input_sync(pidev->input);

	mutex_unlock(&chip->lock);
}

static void ami306_open(struct input_polled_dev *pidev)
{
	struct ami306_chip *chip = pidev->private;
	ami306_add_users(chip);
	ami306_poll(pidev);
}

static void ami306_close(struct input_polled_dev *pidev)
{
	struct ami306_chip *chip = pidev->private;
	ami306_remove_users(chip);
}

int ami306_inputdev_enable(struct ami306_chip *chip)
{
	struct input_dev *input_dev;
	int err;

	if (chip->idev)
		return -EINVAL;

	chip->idev = input_allocate_polled_device();
	if (!chip->idev)
		return -ENOMEM;

	chip->idev->private	  = chip;
	chip->idev->poll	  = ami306_poll;
	chip->idev->close	  = ami306_close;
	chip->idev->open	  = ami306_open;
	chip->idev->poll_interval = AMI306_POLL_INTERVAL;

	input_dev	      = chip->idev->input;
#if 0
	input_dev->name	      = "AMI306 / AK8974 Magnetometer";
#else
	input_dev->name	      = "magnetometer";
#endif
	input_dev->phys	      = DRV_NAME "/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = 0;
	input_dev->dev.parent = &chip->client->dev;

	set_bit(EV_ABS, input_dev->evbit);
	ami306_set_input_params(chip, AMI306_DEFAULT_FUZZ);

	err = input_register_polled_device(chip->idev);
	if (err) {
		input_free_polled_device(chip->idev);
		chip->idev = NULL;
	}

	return err;
}

static int __devinit ami306_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ami306_chip *chip;
	int err = 0;
	int whoami;

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client  = client;

	chip->max_range = AMI306_MAX_RANGE;

	mutex_init(&chip->lock);
	mutex_init(&chip->users_lock);

	whoami = ami306_read(chip, AMI306_WHOAMI);
	if (whoami < 0) {
		dev_err(&client->dev, "device not found\n");
		err = -ENODEV;
		goto fail1;
	}

	if (whoami == AMI306_WHOAMI_VALUE_AMI305) {
		dev_dbg(&client->dev, "device is AMI305, ok\n");
	} else if (whoami == AMI306_WHOAMI_VALUE_AK8974) {
		dev_dbg(&client->dev, "device is AK8974, ok\n");
	} else if (whoami == AMI306_WHOAMI_VALUE_AMI306) {
		dev_dbg(&client->dev, "device is AMI306, ok\n");
	} else if (whoami == AMI306_WHOAMI_VALUE_AMI307) {
		dev_dbg(&client->dev, "device is AMI307, ok\n");
	} else {
		dev_err(&client->dev, "device is not AMI305/AMK306/AK8974 (0x%02x)\n", whoami);
		err = -ENODEV;
		goto fail1;
	}

	err = ami306_configure(chip);
	if (err)
		goto fail1;

	err = ami306_inputdev_enable(chip);
	if (err) {
		dev_err(&client->dev, "Cannot setup input device\n");
		goto fail1;
	}

	err = sysfs_create_group(&chip->client->dev.kobj,
				&ami306_attribute_group);
	if (err)
		dev_err(&client->dev, "Sysfs registration failed\n");

	return err;
fail1:
	kfree(chip);
	return err;
}

static int __devexit ami306_remove(struct i2c_client *client)
{
	struct ami306_chip *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&chip->client->dev.kobj,
			&ami306_attribute_group);
	input_unregister_polled_device(chip->idev);
	input_free_polled_device(chip->idev);
	kfree(chip);
	return 0;
}

#if CONFIG_PM
static int ami306_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ami306_chip *chip = i2c_get_clientdata(client);
	ami306_power(chip, AMI306_PWR_OFF);
	return 0;
}

static int ami306_resume(struct i2c_client *client)
{
	struct ami306_chip *chip = i2c_get_clientdata(client);
	ami306_power(chip, AMI306_PWR_ON);
	return 0;
}

static void ami306_shutdown(struct i2c_client *client)
{
	struct ami306_chip *chip = i2c_get_clientdata(client);
	ami306_power(chip, AMI306_PWR_OFF);
}

#else
#define ami306_suspend NULL
#define ami306_shutdown NULL
#define ami306_resume NULL
#endif

static const struct i2c_device_id ami306_id[] = {
	{DRV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ami306_id);

static struct i2c_driver ami306_driver = {
	.driver	 = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = ami306_suspend,
	.shutdown = ami306_shutdown,
	.resume = ami306_resume,
	.probe	= ami306_probe,
	.remove = __devexit_p(ami306_remove),
	.id_table = ami306_id,
};

static int __init ami306_init(void)
{
	return i2c_add_driver(&ami306_driver);
}

static void __exit ami306_exit(void)
{
	i2c_del_driver(&ami306_driver);
}

MODULE_DESCRIPTION("AMI306 3-axis magnetometer driver");
MODULE_AUTHOR("Samu Onkalo, Nokia Corporation");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" DRV_NAME);

module_init(ami306_init);
module_exit(ami306_exit);
