/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/wakelock.h>
#include <linux/sensors_core.h>
#include <linux/mpu6050_input.h>


#define LOG_RESULT_LOCATION(x) \
	pr_err("%s:%s:%d result=%d\n", __FILE__, __func__, __LINE__, x) \

#define CHECK_RESULT(x) \
	do { \
		if (unlikely(x  != 0)) \
			LOG_RESULT_LOCATION(x); \
		result = x; \
	} while (0)

struct motion_int_data {
	unsigned char pwr_mnt[2];
	unsigned char cfg;
	unsigned char accel_cfg;
	unsigned char int_cfg;
	unsigned char smplrt_div;
	bool is_set;
};

struct mpu6050_input_data {
	struct i2c_client *client;
	struct input_dev *input_accel;
	struct input_dev *input_gyro;
	struct motion_int_data mot_data;
	struct mutex mutex;
	struct mpu_platform_data *pdata;
	struct device *accel_sensor_device;
	struct device *gyro_sensor_device;
#ifdef CONFIG_INPUT_MPU6050_POLLING
	struct delayed_work accel_work;
	struct delayed_work gyro_work;
#endif
	struct wake_lock reactive_wake_lock;
	atomic_t accel_enable;
	atomic_t accel_delay;
	atomic_t gyro_enable;
	atomic_t gyro_delay;
	atomic_t reactive_state;
	unsigned long motion_st_time;
	unsigned char gyro_pwr_mgnt[2];
	unsigned char int_pin_cfg;
	u16 enabled_sensors;
	u16 sleep_sensors;
	int current_delay;
	int gyro_bias[3];
	u8 mode;
	s16 acc_cal[3];
	bool factory_mode;
};

static struct mpu6050_input_data *gb_mpu_data;

struct mpu6050_input_cfg {
	int dummy;
};
static int mpu6050_input_activate_devices(struct mpu6050_input_data *data,
					int sensors, bool enable);

int mpu6050_i2c_write(struct i2c_client *i2c_client,
			unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 1);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}

int mpu6050_i2c_read(struct i2c_client *i2c_client,
			unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].buf = data;
	msgs[0].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 1);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}

int mpu6050_i2c_write_single_reg(struct i2c_client *i2c_client,
					unsigned char reg, unsigned char value)
{

	unsigned char data[2];

	data[0] = reg;
	data[1] = value;

	return mpu6050_i2c_write(i2c_client, 2, data);
}

int mpu6050_i2c_read_reg(struct i2c_client *i2c_client,
			 unsigned char reg, unsigned int len,
			 unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (unlikely(NULL == data || NULL == i2c_client))
		return -EINVAL;

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;

	res = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (unlikely(res < 1))
		return res;
	else
		return 0;
}


int mpu6050_i2c_read_fifo(struct i2c_client *i2c_client,
				unsigned short length, unsigned char *data)
{
	int result;
	unsigned short bytes_read = 0;
	unsigned short this_len;

	while (bytes_read < length) {
		this_len = length - bytes_read;

		result =
			mpu6050_i2c_read_reg(i2c_client,
				MPUREG_FIFO_R_W, this_len, &data[bytes_read]);
		if (result)
			return result;
		bytes_read += this_len;
	}

	return 0;
}



static int mpu6050_input_set_mode(struct mpu6050_input_data *data, u8 mode)
{
	int err = 0;
	data->mode = mode;
	pr_info("%s: setting mode to =%x\n",
			__func__, mode);

	if (mode == MPU6050_MODE_SLEEP) {
		err = mpu6050_input_activate_devices(data,
				MPU6050_SENSOR_ACCEL |
					MPU6050_SENSOR_GYRO, false);
	} else if (mode == MPU6050_MODE_NORMAL) {
		err = mpu6050_input_activate_devices(data,
				MPU6050_SENSOR_ACCEL |
					MPU6050_SENSOR_GYRO, true);
	}

	return err;
}
static void mpu6050_input_report_accel_xyz(struct mpu6050_input_data *data)
{
	static u32 sEventSequenceNum =0;
	u8 regs[6];
	s16 raw[3], orien_raw[3];
	int i = 2;
	int result;

	result =
		mpu6050_i2c_read_reg(data->client,
			MPUREG_ACCEL_XOUT_H, 6, regs);

	raw[0] = ((s16) ((s16) regs[0] << 8)) | regs[1];
	raw[1] = ((s16) ((s16) regs[2] << 8)) | regs[3];
	raw[2] = ((s16) ((s16) regs[4] << 8)) | regs[5];

	do {
		orien_raw[i] = data->pdata->orientation[i*3] * raw[0]
			+ data->pdata->orientation[i*3+1] * raw[1]
			+ data->pdata->orientation[i*3+2] * raw[2];
	} while (i-- != 0);

	pr_debug("%s : x = %d, y = %d, z = %d\n", __func__,
		orien_raw[0], orien_raw[1], orien_raw[2]);

	input_report_abs(data->input_accel, ABS_X, orien_raw[0] - data->acc_cal[0]);
	input_report_abs(data->input_accel, ABS_Y, orien_raw[1] - data->acc_cal[1]);
	input_report_abs(data->input_accel, ABS_Z, orien_raw[2] - data->acc_cal[2]);

	//Send a sequence number as both event and in the sync data to make sure the input subsystem doesn't drop any of them
	input_report_abs(data->input_accel, ABS_CNT, sEventSequenceNum);
	input_event(data->input_accel, EV_SYN, SYN_REPORT, sEventSequenceNum++);
}

static void mpu6050_input_report_gyro_xyz(struct mpu6050_input_data *data)
{
	static u32 sEventSequenceNum =0;
	u8 regs[6];
	s16 raw[3], orien_raw[3];
	int i = 2;
	int result;

	result =
		mpu6050_i2c_read_reg(data->client, MPUREG_GYRO_XOUT_H, 6, regs);

	raw[0] = (((s16) ((s16) regs[0] << 8)) | regs[1])
				- (s16)data->gyro_bias[0];
	raw[1] = (((s16) ((s16) regs[2] << 8)) | regs[3])
				- (s16)data->gyro_bias[1];
	raw[2] = (((s16) ((s16) regs[4] << 8)) | regs[5])
				- (s16)data->gyro_bias[2];

	do {
		orien_raw[i] = data->pdata->orientation[i*3] * raw[0]
			+ data->pdata->orientation[i*3+1] * raw[1]
			+ data->pdata->orientation[i*3+2] * raw[2];
	} while (i-- != 0);

	input_report_abs(data->input_gyro, ABS_RX, orien_raw[0]);
	input_report_abs(data->input_gyro, ABS_RY, orien_raw[1]);
	input_report_abs(data->input_gyro, ABS_RZ, orien_raw[2]);

	//Send a sequence number as both event and in the sync data to make sure the input subsystem doesn't drop any of them
	input_report_abs(data->input_gyro, ABS_RUDDER, sEventSequenceNum);
	input_event(data->input_gyro, EV_SYN, SYN_REPORT, sEventSequenceNum++);

}

static irqreturn_t mpu6050_input_irq_thread(int irq, void *dev)
{
	struct mpu6050_input_data *data = (struct mpu6050_input_data *)dev;

	if (data->enabled_sensors & MPU6050_SENSOR_ACCEL)
		mpu6050_input_report_accel_xyz(data);

	if (data->enabled_sensors & MPU6050_SENSOR_GYRO)
		mpu6050_input_report_gyro_xyz(data);
	return IRQ_HANDLED;
}

static int mpu6050_input_set_fsr(struct mpu6050_input_data *data, int fsr)
{
	unsigned char fsr_mask;
	int result;
	unsigned char reg;
	pr_info("%s: setting to full scale range=%d\n",
			__func__, fsr);

	if (fsr <= 2000) {
		fsr_mask = 0x00;
	} else if (fsr <= 4000) {
		fsr_mask = 0x08;
	} else if (fsr <= 8000) {
		fsr_mask = 0x10;
	} else {		/* fsr = [8001, oo) */
		fsr_mask = 0x18;
	}

	result =
		mpu6050_i2c_read_reg(data->client,
			MPUREG_ACCEL_CONFIG, 1, &reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	result =
		mpu6050_i2c_write_single_reg(data->client, MPUREG_ACCEL_CONFIG,
					 reg | fsr_mask);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	return result;
}

static int mpu6050_input_set_fp_mode(struct mpu6050_input_data *data)
{
	unsigned char b;

	/* Resetting the cycle bit and LPA wake up freq */
	mpu6050_i2c_read_reg(data->client, MPUREG_PWR_MGMT_1, 1, &b);
	b &= ~BIT_CYCLE & ~BIT_PD_PTAT;
	mpu6050_i2c_write_single_reg(data->client, MPUREG_PWR_MGMT_1, b);
	mpu6050_i2c_read_reg(data->client, MPUREG_PWR_MGMT_2, 1, &b);
	b &= ~BITS_LPA_WAKE_CTRL;
	mpu6050_i2c_write_single_reg(data->client, MPUREG_PWR_MGMT_2, b);
	/* Resetting the duration setting for fp mode */
	b = (unsigned char)10 / ACCEL_MOT_DUR_LSB;
	mpu6050_i2c_write_single_reg(data->client, MPUREG_ACCEL_MOT_DUR, b);

	return 0;
}

static int mpu6050_input_set_odr(struct mpu6050_input_data *data, int odr)
{
	int result;
	unsigned char b;
#define HACK_TO_HARDCODE_50HZ_SINCE_THIS_FUNCTION_IS_CURRENTLY_INCORRECT 19
	b = (unsigned char)(HACK_TO_HARDCODE_50HZ_SINCE_THIS_FUNCTION_IS_CURRENTLY_INCORRECT);

	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_SMPLRT_DIV, b));

	mpu6050_i2c_read_reg(data->client, MPUREG_PWR_MGMT_1, 1, &b);
	b &= BIT_CYCLE;
	if (b == BIT_CYCLE) {
		pr_info("Accel LP - > FP mode.\n ");
		mpu6050_input_set_fp_mode(data);
	}

	return result;
}

static int mpu6050_input_set_motion_interrupt(struct mpu6050_input_data *data,
				bool enable, bool factory_test)
{
	struct motion_int_data *mot_data = &data->mot_data;
	unsigned char reg;

	if (enable) {
		if (!mot_data->is_set) {
			mpu6050_i2c_read_reg(data->client,
				MPUREG_PWR_MGMT_1, 2,
					mot_data->pwr_mnt);
			mpu6050_i2c_read_reg(data->client,
				MPUREG_CONFIG, 1, &mot_data->cfg);
			mpu6050_i2c_read_reg(data->client,
				MPUREG_ACCEL_CONFIG, 1,
					&mot_data->accel_cfg);
			mpu6050_i2c_read_reg(data->client,
				MPUREG_INT_ENABLE, 1,
					&mot_data->int_cfg);
			mpu6050_i2c_read_reg(data->client,
				MPUREG_SMPLRT_DIV, 1,
					&mot_data->smplrt_div);
		}

		/* 1) initialize */
		mpu6050_i2c_read_reg(data->client, MPUREG_INT_STATUS, 1, &reg);
		pr_info("%s: Initialize motion interrupt : INT_STATUS=%x\n",
			__func__, reg);

		reg = 0x01;
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1, reg);
		msleep(50);

		/* 2. mpu& accel config */
		if (factory_test)
			reg = 0x0; /*260Hz LPF */
		else
			reg = 0x1; /*44Hz LPF */
		mpu6050_i2c_write_single_reg(data->client, MPUREG_CONFIG, reg);
		reg = 0x0; /* Clear Accel Config. */
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_CONFIG, reg);

		/* 3. set motion thr & dur */
		reg = BIT_RAW_RDY_EN;	/* Make the data ready interrupt enable */
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_INT_ENABLE, reg);
		reg = 0x02;	/* Motion Duration =1 ms */
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_MOT_DUR, reg);
		/* Motion Threshold =1mg, based on the data sheet. */
		if (factory_test)
			reg = 0x00;
		else
			reg = 0x03;
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_MOT_THR, reg);

		/* 5. */
		/* Steps to setup the lower power mode for PWM-2 register */
		reg = mot_data->pwr_mnt[1];
		reg |= (BITS_LPA_WAKE_20HZ); /* setup the frequency of wakeup */
		reg |= 0x07; /* put gyro in standby. */
		reg &= ~(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
		mpu6050_i2c_write_single_reg(data->client,
					MPUREG_PWR_MGMT_2, reg);

		reg = 0x1;
		reg |= 0x20; /* Set the cycle bit to be 1. LOW POWER MODE */
		reg &= ~0x08; /* Clear the temp disp bit. */
		mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1,
				reg & ~BIT_SLEEP);
		data->motion_st_time = jiffies;
	} else {
		if (mot_data->is_set) {
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1,
				mot_data->pwr_mnt[0]);
			msleep(50);
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_2,
				mot_data->pwr_mnt[1]);
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_CONFIG,
				mot_data->cfg);
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_CONFIG,
				mot_data->accel_cfg);
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_INT_ENABLE,
				mot_data->int_cfg);
			reg = 0xff; /* Motion Duration =1 ms */
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_MOT_DUR, reg);
			/* Motion Threshold =1mg, based on the data sheet. */
			reg = 0xff;
			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_ACCEL_MOT_THR, reg);
			mpu6050_i2c_read_reg(data->client,
				MPUREG_INT_STATUS, 1, &reg);

			mpu6050_i2c_write_single_reg(data->client,
				MPUREG_SMPLRT_DIV,
				mot_data->smplrt_div);
		}
	}
	mot_data->is_set = enable;
	return 0;
}

static int mpu6050_input_set_irq(struct mpu6050_input_data *data, int irq)
{
	int result;

	if (irq)
		CHECK_RESULT(mpu6050_i2c_write_single_reg
			(data->client, MPUREG_INT_PIN_CFG,
				data->int_pin_cfg | BIT_BYPASS_EN));
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_INT_ENABLE, irq));

	return result;
}

static int mpu6050_input_suspend_accel(struct mpu6050_input_data *data)
{
	unsigned char reg;
	int result;

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_PWR_MGMT_2, 1, &reg));

	reg |= (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_2, reg));

	return result;
}

static int mpu6050_input_resume_accel(struct mpu6050_input_data *data)
{
	int result;
	unsigned char reg;
	pr_info("%s:\n", __func__);

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_PWR_MGMT_1, 1, &reg));

	if (reg & BIT_SLEEP) {
		CHECK_RESULT(mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1,
				reg & ~BIT_SLEEP));
	}

	usleep_range(2000, 2100);

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_PWR_MGMT_2, 1, &reg));

	reg &= ~(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_2, reg));

	/* settings */

	/*----- LPF configuration  : 188hz ---->*/
// TODO this should be a r/m/w operation because gyro config messes with this register too
	reg = MPU_FILTER_188HZ;
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_CONFIG, reg));
	/*<----- LPF configuration  : 188hz ---- */

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_ACCEL_CONFIG, 1, &reg));
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_ACCEL_CONFIG, reg | 0x0));
	CHECK_RESULT(mpu6050_input_set_fsr(data, 4000));

	return result;
}

static int mpu6050_input_actiave_accel(struct mpu6050_input_data *data,
				bool enable)
{
	int result;

	if (enable) {
		result = mpu6050_input_resume_accel(data);
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		} else {
			data->enabled_sensors |= MPU6050_SENSOR_ACCEL;
		}
	} else {
		result = mpu6050_input_suspend_accel(data);
		if (result == 0)
			data->enabled_sensors &= ~MPU6050_SENSOR_ACCEL;
	}

	return result;
}

static int mpu6050_input_suspend_gyro(struct mpu6050_input_data *data)
{
	int result;

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_PWR_MGMT_1, 2, data->gyro_pwr_mgnt));

	data->gyro_pwr_mgnt[1] |=
		(BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);

	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_2, data->gyro_pwr_mgnt[1]));

	return result;
}

static int mpu6050_input_resume_gyro(struct mpu6050_input_data *data)
{
	int result;
	unsigned regs[2] = { 0, };
	pr_info("%s:\n", __func__);

	CHECK_RESULT(mpu6050_i2c_read_reg
		(data->client, MPUREG_PWR_MGMT_1, 2, data->gyro_pwr_mgnt));

	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_1,
			data->gyro_pwr_mgnt[0] & ~BIT_SLEEP));

	data->gyro_pwr_mgnt[1] &= ~(BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);

	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_2, data->gyro_pwr_mgnt[1]));

	regs[0] = 3 << 3;	/* 2000dps */
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_GYRO_CONFIG, regs[0]));

// TODO this should be a r/m/w operation because XL config messes with this register too
	regs[0] = MPU_FILTER_188HZ | 0x18;
	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_CONFIG, regs[0]));

	return result;
}

static int mpu6050_input_activate_gyro(struct mpu6050_input_data *data,
				bool enable)
{
	int result;
	if (enable) {
		result = mpu6050_input_resume_gyro(data);
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		} else {
			data->enabled_sensors |= MPU6050_SENSOR_GYRO;
		}
	} else {
		result = mpu6050_input_suspend_gyro(data);
		if (result == 0)
			data->enabled_sensors &= ~MPU6050_SENSOR_GYRO;
	}

	return result;
}

static int mpu6050_set_delay(struct mpu6050_input_data *data)
{
	int result;
	int delay = 200;

	if (data->enabled_sensors & MPU6050_SENSOR_ACCEL)
		delay = MIN(delay, atomic_read(&data->accel_delay));

	if (data->enabled_sensors & MPU6050_SENSOR_GYRO)
		delay = MIN(delay, atomic_read(&data->gyro_delay));

	data->current_delay = delay;

	CHECK_RESULT(mpu6050_input_set_odr(data, data->current_delay));
	CHECK_RESULT(mpu6050_input_set_irq(data, BIT_RAW_RDY_EN));

	return result;
}

static int mpu6050_input_activate_devices(struct mpu6050_input_data *data,
					int sensors, bool enable)
{
	int result;
	unsigned char reg;

	if (sensors & MPU6050_SENSOR_ACCEL)
		CHECK_RESULT(mpu6050_input_actiave_accel(data, enable));

	if (sensors & MPU6050_SENSOR_GYRO)
		CHECK_RESULT(mpu6050_input_activate_gyro(data, enable));

	if (data->enabled_sensors) {
		CHECK_RESULT(mpu6050_set_delay(data));
	} else {
		CHECK_RESULT(mpu6050_input_set_irq(data, 0x0));

		CHECK_RESULT(mpu6050_i2c_read_reg
			(data->client, MPUREG_PWR_MGMT_1, 1, &reg));
		if (!(reg & BIT_SLEEP)) {
			CHECK_RESULT(mpu6050_i2c_write_single_reg(data->client,
				MPUREG_PWR_MGMT_1,
				reg |
				BIT_SLEEP));
		}
	}

	return result;
}

static int mpu6050_input_initialize(struct mpu6050_input_data *data,
					const struct mpu6050_input_cfg
					*cfg)
{
	int result;
	data->int_pin_cfg = 0x10;
	data->current_delay = -1;
	data->enabled_sensors = 0;

	CHECK_RESULT(mpu6050_i2c_write_single_reg
		(data->client, MPUREG_PWR_MGMT_1, BIT_H_RESET));
	msleep(30);

	return mpu6050_input_set_mode(data, MPU6050_MODE_SLEEP);
}


int read_accel_raw_xyz(s16 *x, s16 *y, s16 *z)
{
	u8 regs[6];
	s16 raw[3], orien_raw[3];
	int i = 2;
	int result;

	result =
		mpu6050_i2c_read_reg(gb_mpu_data->client,
			MPUREG_ACCEL_XOUT_H, 6, regs);

	raw[0] = ((s16) ((s16) regs[0] << 8)) | regs[1];
	raw[1] = ((s16) ((s16) regs[2] << 8)) | regs[3];
	raw[2] = ((s16) ((s16) regs[4] << 8)) | regs[5];

	do {
		orien_raw[i] = gb_mpu_data->pdata->orientation[i*3] * raw[0]
			+ gb_mpu_data->pdata->orientation[i*3+1] * raw[1]
			+ gb_mpu_data->pdata->orientation[i*3+2] * raw[2];
	} while (i-- != 0);

	pr_info("%s : x = %d, y = %d, z = %d\n", __func__,
		orien_raw[0], orien_raw[1], orien_raw[2]);

	*x = orien_raw[0];
	*y = orien_raw[1];
	*z = orien_raw[2];

	return 0;
}

int read_accel_raw_xyz_cal(s16 *x, s16 *y, s16 *z)
{
	s16 raw_x;
	s16 raw_y;
	s16 raw_z;
	if (!(gb_mpu_data->enabled_sensors & MPU6050_SENSOR_ACCEL)) {
		mpu6050_input_resume_accel(gb_mpu_data);
		usleep_range(10000, 11000);
	}

	read_accel_raw_xyz(&raw_x, &raw_y, &raw_z);
	*x = raw_x - gb_mpu_data->acc_cal[0];
	*y = raw_y - gb_mpu_data->acc_cal[1];
	*z = raw_z - gb_mpu_data->acc_cal[2];

	if (!(gb_mpu_data->enabled_sensors & MPU6050_SENSOR_ACCEL)) {
		mpu6050_input_suspend_accel(gb_mpu_data);
		usleep_range(10000, 11000);
	}
	return 0;
}





static ssize_t mpu6050_input_accel_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_enable));

}

static ssize_t mpu6050_input_accel_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	pr_info("%s : enable = %ld\n", __func__, value);

	mutex_lock(&data->mutex);

	mpu6050_input_activate_devices(data, MPU6050_SENSOR_ACCEL,
			(value == 1) ? true : false);
#ifdef CONFIG_INPUT_MPU6050_POLLING
	if ((atomic_read(&data->accel_enable)) && !value)
		cancel_delayed_work_sync(&data->accel_work);

	if (!(atomic_read(&data->accel_enable)) && value)
		schedule_delayed_work(&data->accel_work, 0);
#endif
	atomic_set(&data->accel_enable, value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6050_input_accel_delay_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->accel_delay));

}

static ssize_t mpu6050_input_accel_delay_store(struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	pr_info("%s : delay = %ld\n", __func__, value);

	mutex_lock(&data->mutex);

	atomic_set(&data->accel_delay, value/1000);

	mpu6050_set_delay(data);

	mutex_unlock(&data->mutex);

	return count;
}



static ssize_t mpu6050_input_gyro_enable_show(struct device *dev,
					struct device_attribute *attr,
						char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_enable));

}

static ssize_t mpu6050_input_gyro_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	pr_info("%s : enable = %ld\n", __func__, value);

	mpu6050_input_activate_devices(data, MPU6050_SENSOR_GYRO,
				(value == 1) ? true : false);
#ifdef CONFIG_INPUT_MPU6050_POLLING
	if ((atomic_read(&data->gyro_enable)) && !value)
		cancel_delayed_work_sync(&data->gyro_work);

	if (!(atomic_read(&data->gyro_enable)) && value)
		schedule_delayed_work(&data->gyro_work, 0);
#endif
	atomic_set(&data->gyro_enable, value);

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mpu6050_input_gyro_delay_show(struct device *dev,
					struct device_attribute *attr,
						char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n", atomic_read(&data->gyro_delay));

}

static ssize_t mpu6050_input_gyro_delay_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct mpu6050_input_data *data = input_get_drvdata(input_data);

	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->mutex);

	atomic_set(&data->gyro_delay, value/1000);

	pr_info("%s : delay = %ld\n", __func__, value);

	mpu6050_set_delay(data);

	mutex_unlock(&data->mutex);

	return count;
}



static DEVICE_ATTR(acc_enable, S_IRUGO | S_IWUGO,
		mpu6050_input_accel_enable_show,
			mpu6050_input_accel_enable_store);
static DEVICE_ATTR(acc_delay, S_IRUGO | S_IWUGO,
		mpu6050_input_accel_delay_show,
			mpu6050_input_accel_delay_store);
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUGO,
		mpu6050_input_gyro_enable_show,
			mpu6050_input_gyro_enable_store);
static DEVICE_ATTR(gyro_delay, S_IRUGO | S_IWUGO,
		mpu6050_input_gyro_delay_show,
			mpu6050_input_gyro_delay_store);



static struct attribute *mpu6050_accel_attributes[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_delay.attr,
	NULL,
};

static struct attribute *mpu6050_gyro_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	NULL,
};

static struct attribute_group mpu6050_accel_attribute_group = {
	.attrs = mpu6050_accel_attributes
};

static struct attribute_group mpu6050_gyro_attribute_group = {
	.attrs = mpu6050_gyro_attributes
};

static ssize_t mpu6050_input_reactive_enable_show(struct device *dev,
					struct device_attribute
						*attr, char *buf)
{
	return sprintf(buf, "%d\n",
		atomic_read(&gb_mpu_data->reactive_state));
}

static ssize_t mpu6050_input_reactive_enable_store(struct device *dev,
					struct device_attribute
						*attr, const char *buf,
							size_t count)
{
	bool onoff = false;
	bool factory_test = false;
	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	if (value == 1) {
		onoff = true;
	} else if (value == 0) {
		onoff = false;
	} else if (value == 2) {
		onoff = true;
		factory_test = true;
		gb_mpu_data->factory_mode = true;
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#ifdef CONFIG_INPUT_MPU6050_POLLING
	if (!value) {
		disable_irq_wake(gb_mpu_data->client->irq);
		disable_irq(gb_mpu_data->client->irq);
	} else {
		enable_irq(gb_mpu_data->client->irq);
		enable_irq_wake(gb_mpu_data->client->irq);
	}
#endif

	mutex_lock(&gb_mpu_data->mutex);
	atomic_set(&gb_mpu_data->reactive_state, onoff);
	mpu6050_input_set_motion_interrupt(gb_mpu_data, onoff, factory_test);
	mutex_unlock(&gb_mpu_data->mutex);

	return count;
}
static struct device_attribute dev_attr_reactive_alert =
	__ATTR(reactive_alert, S_IRUGO | S_IWUSR | S_IWGRP,
		mpu6050_input_reactive_enable_show,
			mpu6050_input_reactive_enable_store);

static ssize_t mpu6050_power_on(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	pr_info("%s n", __func__);

	dev_dbg(dev, "this_client = %d\n", (int)gb_mpu_data->client);
	count = sprintf(buf, "%d\n",
		(gb_mpu_data->client != NULL ? 1 : 0));

	return count;
}
static struct device_attribute dev_attr_power_on =
	__ATTR(power_on, S_IRUSR | S_IRGRP, mpu6050_power_on,
		NULL);

static ssize_t mpu6050_get_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	short temperature = 0;
	unsigned char reg[2];
	int result;

	CHECK_RESULT(mpu6050_i2c_read_reg
		(gb_mpu_data->client, MPUREG_TEMP_OUT_H, 2, reg));

	temperature = (short) (((reg[0]) << 8) | reg[1]);
	temperature = (((temperature + 521) / 340) + 35);

	pr_info("read temperature = %d\n", temperature);

	count = sprintf(buf, "%d\n", temperature);

	return count;
}
static struct device_attribute dev_attr_temperature =
	__ATTR(temperature, S_IRUSR | S_IRGRP, mpu6050_get_temp,
		NULL);

static ssize_t acc_data_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	s16 x;
	s16 y;
	s16 z;
	read_accel_raw_xyz_cal(&x, &y, &z);
	pr_info("%s : x = %d, y = %d, z = %d\n", __func__, x, y, z);
	return sprintf(buf, "%d, %d, %d\n", x, y, z);
}
static struct device_attribute dev_attr_raw_data =
	__ATTR(raw_data, S_IRUSR | S_IRGRP, acc_data_read,
		NULL);


static ssize_t accel_vendor_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "INVENSENSE");
}
static struct device_attribute dev_attr_accel_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_vendor =
	__ATTR(vendor, S_IRUSR | S_IRGRP, accel_vendor_show, NULL);

static ssize_t accel_name_show(struct device *dev,
				struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", "MPU6050");
}
static struct device_attribute dev_attr_accel_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);
static struct device_attribute dev_attr_gyro_sensor_name =
	__ATTR(name, S_IRUSR | S_IRGRP, accel_name_show, NULL);

static struct device_attribute *gyro_sensor_attrs[] = {
	&dev_attr_power_on,
	&dev_attr_temperature,
	&dev_attr_gyro_sensor_vendor,
	&dev_attr_gyro_sensor_name,
	NULL,
};

static struct device_attribute *accel_sensor_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_reactive_alert,
	&dev_attr_accel_sensor_vendor,
	&dev_attr_accel_sensor_name,
	NULL,
};

static int mpu6050_input_register_input_devices(struct
				mpu6050_input_data
					*data)
{
	struct input_dev *idev_accel;
	struct input_dev *idev_gyro;
	int error = 0;

	idev_accel = input_allocate_device();
	if (!idev_accel ) {
		error = -ENOMEM;
		goto done;
	}

	idev_gyro = input_allocate_device();
	if (!idev_gyro) {
		input_free_device(idev_accel);
		error = -ENOMEM;
		goto done;
	}

	idev_accel->name = "mpu6050_accelerometer";
 	idev_accel->id.bustype = BUS_I2C;

	idev_gyro->name = "mpu6050_gyroscope";
	idev_gyro->id.bustype = BUS_I2C;




	set_bit(EV_ABS, idev_accel->evbit);
	input_set_capability(idev_accel, EV_ABS, ABS_X);
	input_set_capability(idev_accel, EV_ABS, ABS_Y);
	input_set_capability(idev_accel, EV_ABS, ABS_Z);

    input_set_abs_params(idev_accel, ABS_X, -2048, 2047, 0, 0);
    input_set_abs_params(idev_accel, ABS_Y, -2048, 2047, 0, 0);
    input_set_abs_params(idev_accel, ABS_Z, -2048, 2047, 0, 0);

	input_set_capability(idev_accel, EV_ABS, ABS_CNT);
	input_set_abs_params(idev_accel, ABS_CNT, -2048, 2047, 0, 0);

	set_bit(EV_ABS, idev_gyro->evbit);
	input_set_capability(idev_gyro, EV_ABS, ABS_RX);
	input_set_capability(idev_gyro, EV_ABS, ABS_RY);
	input_set_capability(idev_gyro, EV_ABS, ABS_RZ);
    input_set_abs_params(idev_gyro, ABS_X, -2048, 2047, 0, 0);
    input_set_abs_params(idev_gyro, ABS_Y, -2048, 2047, 0, 0);
    input_set_abs_params(idev_gyro, ABS_Z, -2048, 2047, 0, 0);

	input_set_capability(idev_gyro, EV_ABS, ABS_RUDDER);
	input_set_abs_params(idev_gyro, ABS_RUDDER, -2048, 2047, 0, 0);


	input_set_drvdata(idev_accel, data);
	input_set_drvdata(idev_gyro, data);

	error  = input_register_device(idev_accel);
	error |= input_register_device(idev_gyro);
	if (error < 0) {
		input_free_device(idev_accel);
		input_free_device(idev_gyro);
		goto done;
	}

	error = sysfs_create_group(&idev_accel->dev.kobj, &mpu6050_accel_attribute_group);
	if (error < 0) {
		input_unregister_device(idev_accel);
		input_unregister_device(idev_gyro);
		goto done;
	}
	error = sysfs_create_group(&idev_gyro->dev.kobj, &mpu6050_gyro_attribute_group);
	if (error < 0) {
		input_unregister_device(idev_accel);
		input_unregister_device(idev_gyro);
		goto done;
	}

	mutex_init(&data->mutex);

	atomic_set(&data->accel_enable, 0);
	atomic_set(&data->accel_delay, 20);
	atomic_set(&data->reactive_state, 0);
	atomic_set(&data->gyro_enable, 0);
	atomic_set(&data->gyro_delay, 20);

	data->input_accel = idev_accel;
	data->input_gyro  = idev_gyro;
done:
	return error;
}

#ifdef CONFIG_INPUT_MPU6050_POLLING
static void mpu6050_work_func_acc(struct work_struct *work)
{
	struct mpu6050_input_data *data =
		container_of((struct delayed_work *)work,
			struct mpu6050_input_data, accel_work);

	mpu6050_input_report_accel_xyz(data);

	pr_debug("%s: enable=%d, delay=%d\n", __func__,
		atomic_read(&data->accel_enable),
			atomic_read(&data->accel_delay));
	if (atomic_read(&data->accel_enable))
		schedule_delayed_work(&data->accel_work,
			msecs_to_jiffies(atomic_read(&data->accel_delay)));
}

static void mpu6050_work_func_gyro(struct work_struct *work)
{
	struct mpu6050_input_data *data =
		container_of((struct delayed_work *)work,
			struct mpu6050_input_data, gyro_work);

	mpu6050_input_report_gyro_xyz(data);

	pr_debug("%s: enable=%d, delay=%d\n", __func__,
		atomic_read(&data->gyro_enable),
			atomic_read(&data->gyro_delay));
	if (atomic_read(&data->gyro_enable))
		schedule_delayed_work(&data->gyro_work,
			msecs_to_jiffies(atomic_read(&data->gyro_delay)));
}
#endif

static int __devinit mpu6050_input_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{

	const struct mpu6050_input_cfg *cfg;
	struct mpu6050_input_data *data;
	int error;
	unsigned char reg;

	pr_info("%s \n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	data = kzalloc(sizeof(struct mpu6050_input_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->pdata  = client->dev.platform_data;

	gb_mpu_data = data;

	error = mpu6050_i2c_read_reg(data->client,
		MPUREG_WHOAMI, 1, &reg);
	if (error < 0) {
		pr_err("%s failed : threre is no such device.\n", __func__);
		goto err_free_mem;
	}
	pr_info("%s: WHOAMI (0x68) = 0x%x\n", __func__, reg);

	error = sensors_register(data->accel_sensor_device,
		NULL, accel_sensor_attrs,
			"accelerometer_sensor");
	if (error) {
		pr_err("%s: cound not register accelerometer sensor device(%d).\n",
			__func__, error);
		goto acc_sensor_register_failed;
	}

	error = sensors_register(data->gyro_sensor_device,
		NULL, gyro_sensor_attrs,
			"gyro_sensor");
	if (error) {
		pr_err("%s: cound not register gyro sensor device(%d).\n",
			__func__, error);
		goto gyro_sensor_register_failed;
	}

#ifdef CONFIG_INPUT_MPU6050_POLLING
        pr_info("mpu6050 configured for polling\n");
	INIT_DELAYED_WORK(&data->accel_work, mpu6050_work_func_acc);
	INIT_DELAYED_WORK(&data->gyro_work, mpu6050_work_func_gyro);
#endif

	wake_lock_init(&data->reactive_wake_lock, WAKE_LOCK_SUSPEND,
		"reactive_wake_lock");

	error = mpu6050_input_initialize(data, cfg);
	error = mpu6050_input_register_input_devices(data);
        if (error < 0)
          goto err_input_initialize;

	if (error)
		goto err_input_initialize;

	if (client->irq > 0) {
		error = request_threaded_irq(client->irq,
				NULL, mpu6050_input_irq_thread,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				MPU6050_INPUT_DRIVER, data);
		if (error < 0) {
			dev_err(&client->dev,
				"irq request failed %d, error %d\n",
				client->irq, error);
			input_unregister_device(data->input_accel);
			input_unregister_device(data->input_gyro);
			goto err_input_initialize;
		}
#ifdef CONFIG_INPUT_MPU6050_POLLING
		disable_irq(client->irq);
#endif
	}

	i2c_set_clientdata(client, data);

	pr_info("mpu6050_input_probe success\n");

	return 0;

err_input_initialize:
	wake_lock_destroy(&data->reactive_wake_lock);
gyro_sensor_register_failed:
acc_sensor_register_failed:
err_free_mem:
	kfree(data);
	return error;
}

static int mpu6050_input_remove(struct i2c_client *client)
{
	struct mpu6050_input_data *data = i2c_get_clientdata(client);

	if (client->irq > 0) {
		free_irq(client->irq, data);
		input_unregister_device(data->input_accel);
		input_unregister_device(data->input_gyro);
		// TODO shouldn't this also do a kfree on the devices?
	}
	wake_lock_destroy(&data->reactive_wake_lock);
	kfree(data);

	return 0;
}

static int mpu6050_input_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6050_input_data *data = i2c_get_clientdata(client);
	pr_info("%s: suspending\n", __func__);

	if (!atomic_read(&data->reactive_state)) {
#ifdef CONFIG_INPUT_MPU6050_POLLING
		if (atomic_read(&data->accel_enable))
			cancel_delayed_work_sync(&data->accel_work);
		if (atomic_read(&data->gyro_enable))
			cancel_delayed_work_sync(&data->gyro_work);
#else
		disable_irq_wake(client->irq);
		disable_irq(client->irq);
#endif
		mpu6050_input_set_mode(data, MPU6050_MODE_SLEEP);
	}
	return 0;
}

static int mpu6050_input_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6050_input_data *data = i2c_get_clientdata(client);
	pr_info("%s: resuming\n", __func__);

	if (!atomic_read(&data->reactive_state)) {
		mpu6050_input_set_mode(data, MPU6050_MODE_NORMAL);
#ifdef CONFIG_INPUT_MPU6050_POLLING
		if (atomic_read(&data->accel_enable))
			schedule_delayed_work(&data->accel_work, 0);
		if (atomic_read(&data->gyro_enable))
			schedule_delayed_work(&data->gyro_work, 0);
#else
		enable_irq(client->irq);
		enable_irq_wake(client->irq);
#endif
	}
	return 0;
}

static const struct i2c_device_id mpu6050_input_id[] = {
	{MPU6050_INPUT_DRIVER, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpu6050_input_id);

static const struct dev_pm_ops mpu6050_dev_pm_ops = {
	.suspend = mpu6050_input_suspend,
	.resume = mpu6050_input_resume,
};

static struct i2c_driver mpu6050_input_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = MPU6050_INPUT_DRIVER,
		.pm = &mpu6050_dev_pm_ops,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = mpu6050_input_id,
	.probe = mpu6050_input_probe,
	.remove = mpu6050_input_remove,
};

static int __init mpu6050_init(void)
{
	int result = i2c_add_driver(&mpu6050_input_driver);

	pr_info("my %s \n", __func__);

	return result;
}

static void __exit mpu6050_exit(void)
{
	pr_info("%s \n", __func__);

	i2c_del_driver(&mpu6050_input_driver);
}

MODULE_AUTHOR("Tae-Soo Kim <tskim@invensense.com>");
MODULE_DESCRIPTION("MPU6050 driver");
MODULE_LICENSE("GPL");

module_init(mpu6050_init);
module_exit(mpu6050_exit);
