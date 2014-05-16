/*
    sensor_relay.c - Linux kernel module for publishing sensor
    measuremnents using RELAY

    This file declares helper functions for the sysfs class "sensor_relay",
    for use by sensors drivers.

    Copyright (C) 2013 <sungerfeld@sensorplatforms.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/idr.h>
#include <linux/sensor_relay.h>
#include <linux/gfp.h>
#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/input.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/ctype.h>

#define SENSOR_RELAY_ID_PREFIX "sensor_relay"
#define SENSOR_RELAY_ID_FORMAT  "%s"

static struct class *sensor_relay_class;
static unsigned long registered_sensor_active;


static bool sensor_relay_buf_mapped_by_user;
static struct rchan *sensor_relay_chan;
static struct input_dev *sensor_relay_input;
static bool sensor_relay_opened;
static uint32_t sensor_relay_wakeup_flag = 0x0555aaaa;
static struct dentry *sensor_relay_produced_control[NR_CPUS];
static struct dentry *sensor_relay_consumed_control[NR_CPUS];
static unsigned int sensor_relay_node_count;




static struct rchan *sensor_relay_chan;

static DEFINE_IDR(
        sensor_relay_idr);
static DEFINE_SPINLOCK(
        idr_lock);

static DECLARE_RWSEM(
        sensor_relay_list_lock);

static LIST_HEAD(
        sensor_relay_list);


/**
 * sensor_relay_device_register - register w/ sensor_relay
 * @dev: the device to register
 *
 * sensor_relay_device_unregister() must be called when the device is no
 * longer needed.
 *
 * Returns the pointer to the new device.
 */




static int sensor_relay_input_open(
        struct input_dev *dev) {
    sensor_relay_opened = true;
    return 0;
}


static void sensor_relay_input_close(
        struct input_dev *dev) {
    sensor_relay_opened = false;
}


/*
 * sensor_relay_subbuf_start() relayfs callback.
 *
 * Defined so that we can 1) reserve padding counts in the sub-buffers, and
 * 2) keep a count of events dropped due to the buffer-full condition.
 */
static int sensor_relay_subbuf_start_callback(
        struct rchan_buf *buf,
        void *subbuf,
        void *prev_subbuf,
        unsigned int prev_padding) {


    if (relay_buf_full(buf)) {
        printk(KERN_ERR
               "sensor data lost - CPU %d consumed sub buffers %d"
               "consumed %d\n",
               buf->cpu, buf->subbufs_produced, buf->subbufs_consumed);

        return 0;
    }
    return 1;
}

static int sensor_relay_remove_buf_file_callback(
        struct dentry *dentry) {
    debugfs_remove(dentry);

    return 0;
}

static struct dentry *sensor_relay_create_buf_file_callback(
        const char *filename,
        struct dentry *parent,
        int mode,
        struct rchan_buf *buf,
        int *is_global) {
    return debugfs_create_file(filename, mode, parent, buf,
                               &relay_file_operations);
}

static void sensor_relay_buf_mapped(
        struct rchan_buf *buf,
        struct file *filp) {

    sensor_relay_buf_mapped_by_user = true;

    buf->subbufs_produced = 0;
    buf->subbufs_consumed = 0;
    buf->bytes_consumed = 0;
    buf->finalized = 0;
    buf->data = buf->start;
    buf->offset = 0;
}

static void sensor_relay_buf_unmapped(
        struct rchan_buf *buf,
        struct file *filp) {
    sensor_relay_buf_mapped_by_user = false;
    printk(KERN_ERR
           "sensor_relay_buf_unmapped\n");
}

/*
 * relayfs callbacks
 */

static struct rchan_callbacks sensor_relay_callbacks = {
    .subbuf_start = sensor_relay_subbuf_start_callback,
    .create_buf_file = sensor_relay_create_buf_file_callback,
    .remove_buf_file = sensor_relay_remove_buf_file_callback,
    .buf_mapped = sensor_relay_buf_mapped,
    .buf_unmapped = sensor_relay_buf_unmapped,
};


static int sensor_relay_consumed_open(
        struct inode *inode,
        struct file *filp) {
    filp->private_data = inode->i_private;
    printk(KERN_ERR
           "sensor_relay_consumed_open\n");
    return 0;
}

static ssize_t sensor_relay_consumed_read(
        struct file *filp,
        char __user * buffer,
        size_t count,
        loff_t * ppos) {
    struct rchan_buf *buf = filp->private_data;
    printk(KERN_ERR
           "sensor_relay_consumed_read\n");

    return simple_read_from_buffer(buffer, count, ppos,
                                   &buf->subbufs_consumed,
                                   sizeof(buf->subbufs_consumed));

}

static ssize_t sensor_relay_consumed_write(
        struct file *filp,
        const char __user * buffer,
        size_t count,
        loff_t * ppos) {
    struct rchan_buf *buf = filp->private_data;

    size_t consumed;

    if (copy_from_user(&consumed, buffer, sizeof(consumed)))
        return -EFAULT;

    relay_subbufs_consumed(buf->chan, buf->cpu, consumed);
#if 0
    printk(KERN_ERR
           "received CPU %d consumed sub buffers %d   consumed %d"
           "reported %d\n",
           buf->cpu, consumed, buf->subbufs_consumed,
           buf->subbufs_produced);
#endif
    return count;
}

/*
 * 'consumed' file operations - r/w, binary
 *
 *  There is a .consumed file associated with each per-cpu relay file.
 *  Writing to a .consumed file adds the value written to the
 *  subbuffers-consumed count of the associated relay buffer.
 *  Reading a .consumed file returns the number of sub-buffers so far
 *  consumed for the associated relay buffer.
 */
const struct file_operations sensor_relay_consumed_fops = {
    .owner = THIS_MODULE,
    .open = sensor_relay_consumed_open,
    .read = sensor_relay_consumed_read,
    .write = sensor_relay_consumed_write,
};


/*
 * control files for relay produced/consumed sub-buffer counts
 */

static int sensor_relay_produced_open(
        struct inode *inode,
        struct file *filp) {
    int i;

    struct rchan_buf *buf = inode->i_private;

    filp->private_data = inode->i_private;

    buf->subbufs_produced = 0;
    buf->subbufs_consumed = 0;
    buf->bytes_consumed = 0;
    buf->finalized = 0;
    buf->data = buf->start;
    buf->offset = 0;


    for (i = 0; i < buf->chan->n_subbufs; i++)
        buf->padding[i] = 0;

    buf->chan->cb->subbuf_start(buf, buf->data, NULL, 0);
    return 0;
}

static ssize_t sensor_relay_produced_read(
        struct file *filp,
        char __user * buffer,
        size_t count,
        loff_t * ppos) {
    struct rchan_buf *buf = filp->private_data;

    ssize_t err = simple_read_from_buffer(buffer, count, ppos,
                                          &buf->subbufs_produced,
                                          sizeof(buf->subbufs_produced));

#if 0
    printk(KERN_ERR
           "reported CPU %d produced sub buffers %d   consumed %d dest %p"
           "offset %lld\n",
           buf->cpu, buf->subbufs_produced, buf->subbufs_consumed, buffer,
           *ppos);
#endif
    return err;
}

/*
 * 'produced' file operations - r, binary
 *
 *  There is a .produced file associated with each per-cpu relay file.
 *  Reading a .produced file returns the number of sub-buffers so far
 *  produced for the associated relay buffer.
 */
const struct file_operations sensor_relay_produced_fops = {
    .owner = THIS_MODULE,
    .open = sensor_relay_produced_open,
    .read = sensor_relay_produced_read,
    .llseek = default_llseek
};



static int sensor_relay_update_enable(
        struct sensor_relay_classdev *sensor_relay_cdev) {
    int err = 0;

    if (sensor_relay_cdev->enable_get)
        err = sensor_relay_cdev->enable_get(sensor_relay_cdev,
                                            &sensor_relay_cdev->enable);
    return err;
}

static int sensor_relay_set_enable(
        struct sensor_relay_classdev *sensor_relay_cdev,
        bool enable) {
    int err = 0;

    if (sensor_relay_cdev->enable_set)
        err = sensor_relay_cdev->enable_set(sensor_relay_cdev, enable);
    return err;
}

static ssize_t sensor_relay_enable_show(
        struct device *dev,
        struct device_attribute *attr,
        char *buf) {
    struct sensor_relay_classdev *sensor_relay_cdev = dev_get_drvdata(dev);

    /*
     * no lock needed for this
     */
    sensor_relay_update_enable(sensor_relay_cdev);

    return snprintf(buf, SENSOR_RELAY_BUFF_SIZE, "%c\n",
        sensor_relay_cdev->enable ? '1' : '0');
}

static ssize_t sensor_relay_enable_store(
        struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t size) {
    struct sensor_relay_classdev *sensor_relay_cdev = dev_get_drvdata(dev);
    ssize_t ret;
    unsigned long state;

    ret = strict_strtoul(buf, 10, &state);
    if (ret == 0) {
        ret = size;
        sensor_relay_set_enable(sensor_relay_cdev, state ? 1 : 0);
    }
    return ret;
}



static int sensor_relay_update_delay(
        struct sensor_relay_classdev *sensor_relay_cdev) {
    int err = 0;

    if (sensor_relay_cdev->delay_get)
        err = sensor_relay_cdev->delay_get(sensor_relay_cdev,
                                           &sensor_relay_cdev->delay);
    return err;
}

static int sensor_relay_set_delay(
        struct sensor_relay_classdev *sensor_relay_cdev,
        unsigned int delay) {
    int err = 0;

    if (sensor_relay_cdev->delay_set)
        err = sensor_relay_cdev->delay_set(sensor_relay_cdev, delay);
    return err;
}

static ssize_t sensor_relay_delay_show(
        struct device *dev,
        struct device_attribute *attr,
        char *buf) {
    struct sensor_relay_classdev *sensor_relay_cdev = dev_get_drvdata(dev);

    /*
     * no lock needed for this
     */
    sensor_relay_update_delay(sensor_relay_cdev);

    return snprintf(buf,
                    SENSOR_RELAY_BUFF_SIZE,
                    "%u\n", sensor_relay_cdev->delay);
}



static ssize_t sensor_relay_delay_store(
        struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t size) {
    struct sensor_relay_classdev *sensor_relay_cdev = dev_get_drvdata(dev);
    ssize_t ret;
    unsigned long state;

    ret = strict_strtoul(buf, 10, &state);

    if (ret == 0) {
        ret = size;
        sensor_relay_set_delay(sensor_relay_cdev, state);
    }
    return ret;
}


static struct device_attribute sensor_relay_class_attrs[] = {
    __ATTR(enable, 0644, sensor_relay_enable_show,
    sensor_relay_enable_store),
    __ATTR(delay, 0644, sensor_relay_delay_show, sensor_relay_delay_store),

    __ATTR_NULL,
};


static int sensor_relay_input_init(
        void) {
    int err = 0;

    sensor_relay_input = input_allocate_device();
    sensor_relay_opened = false;

    if (!sensor_relay_input) {
        printk(KERN_ERR "failed to allocate memory for device\n");
        return -ENOMEM;
    }

    sensor_relay_input->open = sensor_relay_input_open;
    sensor_relay_input->close = sensor_relay_input_close;

    sensor_relay_input->id.bustype = BUS_VIRTUAL;
    sensor_relay_input->name = "sensor_relay_kernel";


    set_bit(EV_ABS, sensor_relay_input->evbit);
    input_set_capability(sensor_relay_input, EV_ABS, ABS_VOLUME);
    input_set_abs_params(sensor_relay_input,
                         ABS_VOLUME, 0x0555aaaa, 0x0aaa5555, 0, 0);
    err = input_register_device(sensor_relay_input);
    if (err) {
        input_free_device(sensor_relay_input);
        printk(KERN_ERR "failed to register input device\n");
    }

    return err;
}


static int sensor_relay_device_init(
        void) {
    int err, i;
    char tmpname[32];

    sensor_relay_class = class_create(THIS_MODULE, "sensor_relay");
    if (IS_ERR(sensor_relay_class)) {
        pr_err("couldn't create sysfs class\n");
        return -EFAULT;
    }
    sensor_relay_class->dev_attrs = sensor_relay_class_attrs;

    registered_sensor_active = 0;


    sensor_relay_chan = NULL;

    printk(KERN_ERR "relay rec size %d  num buffers %d \n",
           sizeof(union  sensor_relay_broadcast_node), SENSOR_RELAY_NUM_RELAY_BUFFERS);

    sensor_relay_chan = relay_open("sensor_relay_kernel",
                                   NULL,
                                   sizeof(union sensor_relay_broadcast_node),
                                   SENSOR_RELAY_NUM_RELAY_BUFFERS,
                                   &sensor_relay_callbacks, NULL);

    if (!sensor_relay_chan) {
        printk("relay channel creation failed.\n");
        err = -EFAULT;
        goto err_destroy_class;
    }

    sensor_relay_buf_mapped_by_user = false;
    sensor_relay_node_count = 0;

    err = sensor_relay_input_init();


    if (err) {
        printk("failed to create input file for sensor relay.\n");
        err = -EFAULT;
        goto err_destroy_channel;
    }
    printk("Create relay control files\n");

    for_each_online_cpu(i) {

        sprintf(tmpname, "sensor_relay_kernel%d.produced", i);
        printk("Create relay control file %s.\n",
               tmpname);
        sensor_relay_produced_control[i] = debugfs_create_file(tmpname,
                                                               S_IRUGO |
                                                               S_IWUGO,
                                                               NULL,
                                                               sensor_relay_chan->
                                                               buf[i],
                                                               &sensor_relay_produced_fops);


        if (!sensor_relay_produced_control[i]) {
            printk("Couldn't create relay control file %s.\n",
                   tmpname);
            goto err_free_control_files;
        }

        sprintf(tmpname, "sensor_relay_kernel%d.consumed", i);
        sensor_relay_consumed_control[i] = debugfs_create_file(tmpname,
                                                               S_IRUGO |
                                                               S_IWUGO,
                                                               NULL,
                                                               sensor_relay_chan->
                                                               buf[i],
                                                               &sensor_relay_consumed_fops);


        if (!sensor_relay_consumed_control[i]) {
            printk("Couldn't create relay control file %s.\n",
                   tmpname);
            goto err_free_control_files;
        }
    }
    return 0;

err_free_control_files:

    for_each_online_cpu(i)
            if (sensor_relay_produced_control[i]) {
        debugfs_remove(sensor_relay_produced_control[i]);
        sensor_relay_produced_control[i] = NULL;
    }


    for_each_online_cpu(i)
            if (sensor_relay_consumed_control[i]) {
        debugfs_remove(sensor_relay_consumed_control[i]);
        sensor_relay_consumed_control[i] = NULL;
    }

err_destroy_channel:
    if (sensor_relay_chan) {
        relay_close(sensor_relay_chan);
        sensor_relay_chan = NULL;
    }

err_destroy_class:
    class_destroy(sensor_relay_class);
    return err;

}

int sensor_relay_device_register(
        struct device *parent,
        struct sensor_relay_classdev
        *sensor_relay_cdev) {
    int id;
    int err = 0;

    if (sensor_relay_class == NULL)
        err = sensor_relay_device_init();

again:
    if (unlikely(idr_pre_get(&sensor_relay_idr, GFP_KERNEL) == 0))
        return -ENOMEM;

    spin_lock(&idr_lock);
    err = idr_get_new(&sensor_relay_idr, NULL, &id);
    spin_unlock(&idr_lock);

    if (unlikely(err == -EAGAIN))
        goto again;
    else if (unlikely(err))
        return err;

    id = id & MAX_ID_MASK;

    sensor_relay_cdev->dev = device_create(sensor_relay_class,
                                           parent,
                                           MKDEV(0, 0),
                                           sensor_relay_cdev,
                                           SENSOR_RELAY_ID_FORMAT,
                                           sensor_relay_cdev->sensor_name);

    if (IS_ERR(sensor_relay_cdev->dev)) {
        spin_lock(&idr_lock);
        idr_remove(&sensor_relay_idr, id);
        spin_unlock(&idr_lock);
    } else
        registered_sensor_active |= (1 << sensor_relay_cdev->sensorId);

    /*
     * add to the list of leds
     */
    down_write(&sensor_relay_list_lock);
    list_add_tail(&sensor_relay_cdev->node, &sensor_relay_list);
    up_write(&sensor_relay_list_lock);

    return 0;
}

EXPORT_SYMBOL_GPL(sensor_relay_device_register);

/**
 * sensor_relay_device_unregister - removes the previously
 * registered class device
 *
 * @dev: the class device to destroy
 */
void sensor_relay_device_unregister(
        struct sensor_relay_classdev
        *sensor_relay_cdev) {
    device_unregister(sensor_relay_cdev->dev);

    down_write(&sensor_relay_list_lock);
    list_del(&sensor_relay_cdev->node);
    up_write(&sensor_relay_list_lock);
}

EXPORT_SYMBOL_GPL(sensor_relay_device_unregister);



int sensor_relay_write(
        struct sensor_relay_classdev *sensor_relay_cdev,
        union sensor_relay_broadcast_node *node) {
    int err = 0;

    if ((sensor_relay_chan != NULL) && (sensor_relay_buf_mapped_by_user)) {
        relay_write(sensor_relay_chan, node, sizeof(*node));
        sensor_relay_node_count++;
    } else
        err = -EFAULT;

    return err;
}

EXPORT_SYMBOL_GPL(sensor_relay_write);



int sensor_relay_wakeup(
        struct sensor_relay_classdev *sensor_relay_cdev) {
    int err = 0;

    if ((sensor_relay_chan != NULL) && (sensor_relay_input != NULL) &&
            (sensor_relay_buf_mapped_by_user)) {

        input_event(sensor_relay_input, EV_ABS, ABS_VOLUME,
                    sensor_relay_wakeup_flag);
        input_sync(sensor_relay_input);

        sensor_relay_wakeup_flag = sensor_relay_wakeup_flag == 0x0555aaaa ?
                    0x0aaa5555 : 0x0555aaaa;
    } else
        err = -EFAULT;

    return err;
}

EXPORT_SYMBOL_GPL(sensor_relay_wakeup);



static int __init sensor_relay_init(
        void) {
    sensor_relay_class = NULL;
    sensor_relay_input = NULL;

    pr_err("sensor_relay_init\n");


    return 0;
}

static void __exit sensor_relay_exit(
        void) {
    class_destroy(sensor_relay_class);
} subsys_initcall(sensor_relay_init);

module_exit(sensor_relay_exit);


MODULE_AUTHOR("Shmuel Ungerfeld <sungerfeld@sensorplatforms.com>");
MODULE_DESCRIPTION("sensor reporting via relay sysfs/class support");
MODULE_LICENSE("GPL");

