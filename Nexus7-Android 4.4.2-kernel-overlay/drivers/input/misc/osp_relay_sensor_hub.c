/*
 * OSP Sensor Hub driver
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include "osp-sensors.h"
#include <linux/sensor_relay.h>

/* Message types from Hub */

#define ARRAY_INDEX(arrElement, arrayStart) (((void *) arrElement) - ((void *)arrayStart) / sizeof((arr)[0]))


uint64_t TimeStamp40_to_timeStamp64(
        struct TimeStamp40 timeStamp) {
    union spi_pack {
        uint64_t int64;
        struct spi_pack {
            uint32_t int32;
            uint8_t int8[4];
        } parts;
    } u;

    u.parts.int32 = timeStamp.timeStamp32;
    u.parts.int8[0] = timeStamp.timeStamp40;
    u.parts.int8[1] = 0;
    u.parts.int8[2] = 0;
    u.parts.int8[3] = 0;

    return u.int64;
}

struct sensorhub_sensor {
    struct i2c_client *client;
    struct device *dev;
    struct work_struct work;
    struct mutex mutex;

    int interruptsEnabled;

    struct sensor_data {
        struct sensor_relay_classdev sensor_relay_cdev;
        unsigned char enabled;
        unsigned long delay;
        struct osp_sh_sensor_broadcast_node last_node;
    } sensor_data[OSP_SH_SENSOR_ID_COUNT];

    char packet[32];
};


#define SENSOR_DATA(sensorId) \
    (sensor->sensor_data[sensorId - OSP_SH_SENSOR_ID_FIRST])
#define LAST_SENSOR_NODE(sensorId)            \
    (SENSOR_DATA(sensorId).last_node.data.sensorData)

#define LAST_CHANGE_DETECTOR_NODE(sensorId)   \
    (SENSOR_DATA(sensorId).last_node.data.segmentData)

#define LAST_QUATERNION_NODE(sensorId)   \
    (SENSOR_DATA(sensorId).last_node.data.quaternionData)


/*******************************************************************************
 * @fn      sensorhub_read_buf
 *          Helper function to read the hub register values in one transaction
 *          or returns a negative error code on failure
 *
 ******************************************************************************/
static int sensorhub_read_buf(struct i2c_client *client, u8 cmd, u8 *buffer, int length)
{
    /*
     * Annoying we can't make this const because the i2c layer doesn't
     * declare input buffers const.
     */
    struct i2c_msg msg[] = {
    {
        .addr = client->addr,
        .flags = 0,
        .len = 1,
        .buf = &cmd,
    },
    {
        .addr = client->addr,
        .flags = I2C_M_RD,
        .len = length,
        .buf = buffer,
    },
};

    return i2c_transfer(client->adapter, msg, 2);
}

/*******************************************************************************
 * @fn      sensorhub_sensor_read_buf
 *          Helper function to read sensor data over I2C bus
 *
 ******************************************************************************/
static int sensorhub_sensor_read_buf(struct i2c_client *client, SensorType_t sensorId , u8 cmd, u8 *buffer, int length)
{
    u8 buf[2] = {cmd, sensorId};
    /*
     * Annoying we can't make this const because the i2c layer doesn't
     * declare input buffers const.
     */
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = sizeof(buf),
            .buf = buf,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = buffer,
        },
    };

    return i2c_transfer(client->adapter, msg, 2);
}


/*******************************************************************************
 * @fn      sensorhub_send_buf
 *          Writes the register values in one transaction or returns a negative
 *          error code on failure.
 *
 ******************************************************************************/
static int sensorhub_send_buf(struct i2c_client *client, u8 *buffer, int length)
{
    /*
     * Annoying we can't make this const because the i2c layer doesn't
     * declare input buffers const.
     */
    struct i2c_msg msg =
    {
        .addr = client->addr,
        .flags = 0,
        .len = length,
        .buf = buffer,
    };

    return i2c_transfer(client->adapter, &msg, 1);
}


/*******************************************************************************
 * @fn      osp_sh_work_func
 *          Sensor Hub interrupt thread worker
 *
 ******************************************************************************/
static void osp_sh_work_func(struct work_struct *work)
{
    union sensor_relay_broadcast_node relay_node;
    struct osp_sh_sensor_broadcast_node node;
    char *ptr;
    SensorType_t sensorId = 0;
    int err;

    struct sensorhub_sensor *sensor =
        container_of(work, struct sensorhub_sensor, work);

    u16 length;

    /*    dev_info(&sensor->client->dev, "osp_sh_work_func Start"); */

    while (!gpio_get_value(TEGRA_IRQ_TO_GPIO(sensor->client->irq))) {

        /* Read interrupt reason and length */
        err = sensorhub_read_buf(sensor->client, OSP_SH_GET_BROADCAST_LENGTH, (u8 *)&length, sizeof(length));
        if ((err < 0) || (length == 0))
            continue;

        /* If valid, read specified length from slave memory */
        err = sensorhub_read_buf(sensor->client, OSP_SH_GET_BROADCAST_DATA, sensor->packet, length);

        for (ptr = sensor->packet;(err >= 0) && (ptr < (sensor->packet + length));) {

            /*
            * printk("[OSP]: sensor: %d  size %d  "
            * "addr: 0x%p\n",
            * *ptr, sizeof(node), ptr);
            */
            sensorId = (*ptr);

            node.sensorId = sensorId;

            switch (sensorId) {
            case SENSOR_ACCELEROMETER_UNCALIBRATED:
            case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            case SENSOR_GYROSCOPE_UNCALIBRATED:
                memcpy(&node.data.uncal_sensorData,
                    ptr + 2,
                    sizeof(node.data.uncal_sensorData));

                memcpy(&LAST_SENSOR_NODE(sensorId),
                    &node.data.uncal_sensorData,
                    sizeof(node.data.uncal_sensorData));

                relay_node.uncal_sensorData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.uncal_sensorData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.uncal_sensorData.timeStamp);

                memcpy(&relay_node.uncal_sensorData.Data,
                    &node.data.uncal_sensorData.Data, sizeof(relay_node.uncal_sensorData.Data));
                memcpy(&relay_node.uncal_sensorData.Bias,
                    &node.data.uncal_sensorData.Bias, sizeof(relay_node.uncal_sensorData.Bias));
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-4.4x%-8.8x",
                    __func__,
                    sensorId,
                    relay_node.uncal_sensorData.timeStamp.timeStamp40,
                    relay_node.uncal_sensorData.timeStamp.timeStamp32);
#endif
                ptr += sizeof(node.data.uncal_sensorData) + 2;
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-16.16llx  x: %-5.4d Y: %-5.4d Z: %-5.4d bx: %-5.4d bY: %-5.4d bZ: %-5.4d ",
                    __func__,
                    relay_node.uncal_sensorData.sensorId,
                    relay_node.uncal_sensorData.timeStamp,
                    relay_node.uncal_sensorData.Data[0],
                    relay_node.uncal_sensorData.Data[1],
                    relay_node.uncal_sensorData.Data[2],
                    relay_node.uncal_sensorData.Bias[0],
                    relay_node.uncal_sensorData.Bias[1],
                    relay_node.uncal_sensorData.Bias[2],
                    );
#endif
                sensor_relay_write
                    (&SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);
                break;

            case SENSOR_ACCELEROMETER_CALIBRATED:
            case SENSOR_GYROSCOPE_CALIBRATED:
            case SENSOR_MAGNETIC_FIELD_CALIBRATED:
            case SENSOR_GRAVITY:
            case SENSOR_LINEAR_ACCELERATION:
                memcpy(&node.data.sensorData,
                    ptr + 2,
                    sizeof(node.data.sensorData));

                memcpy(&LAST_SENSOR_NODE(sensorId),
                    &node.data.sensorData,
                    sizeof(node.data.sensorData));

                relay_node.sensorData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.sensorData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.sensorData.timeStamp);

                memcpy(&relay_node.sensorData.Data,
                    &node.data.sensorData.Data, sizeof(relay_node.sensorData.Data));
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-4.4x%-8.8x",
                    __func__,
                    sensorId,
                    relay_node.sensorData.timeStamp.timeStamp40,
                    sensorData.timeStamp.timeStamp32);
#endif
                ptr += sizeof(node.data.sensorData) + 2;
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-16.16llx  x: %-5.4d Y: %-5.4d Z: %-5.4d ",
                    __func__,
                    relay_node.sensorData.sensorId,
                    relay_node.sensorData.timeStamp,
                    relay_node.sensorData.Data[0],
                    relay_node.sensorData.Data[1],
                    relay_node.sensorData.Data[2]);
#endif
                sensor_relay_write
                    (&SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);
                break;

            case SENSOR_CONTEXT_CHANGE_DETECTOR:

                memcpy(&node.data.segmentData,
                    ptr + 2,
                    sizeof(node.data.segmentData));

                memcpy(&LAST_SENSOR_NODE(sensorId),
                    &node.data.segmentData,
                    sizeof(node.data.segmentData));

                relay_node.segmentData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;


                relay_node.segmentData.duration =
                    node.data.segmentData.duration;

                relay_node.segmentData.endTime =
                    node.data.segmentData.endTime;

                relay_node.segmentData.type =
                    node.data.segmentData.type;

                ptr += sizeof(node.data.segmentData) + 2;

#if 1
                dev_err(&sensor->client->dev,
                    "[OSP]: sensor %d TimeStamp %llu duratuion %u type %d",
                    relay_node.segmentData.sensorId,
                    relay_node.segmentData.endTime,
                    relay_node.segmentData.duration,
                    relay_node.segmentData.type);
#endif
                sensor_relay_write(&SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);
                break;

            case SENSOR_STEP_DETECTOR:
            case SENSOR_STEP_COUNTER:
                memcpy(&node.data.stepSensitiveData,
                    ptr + 2,
                    sizeof(node.data.
                    stepSensitiveData));

                memcpy(&LAST_SENSOR_NODE(sensorId),
                    &node.data.stepSensitiveData,
                    sizeof(node.data.stepSensitiveData));

                relay_node.segmentData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.stepSensitiveData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.sensorData.timeStamp);

                relay_node.stepSensitiveData.numStepsTotal =
                    node.data.stepSensitiveData.numStepsTotal;

                ptr += sizeof(node.data.stepSensitiveData) + 2;

#if 1
                dev_err(&sensor->client->dev,
                    "[OSP]: sensor %d timeStamp %llu numStepsTotal %d ",
                    relay_node.stepSensitiveData.sensorId,
                    relay_node.stepSensitiveData.timeStamp,
                    relay_node.stepSensitiveData.numStepsTotal);
#endif
                sensor_relay_write(&SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);

                break;

            case SENSOR_CONTEXT_DEVICE_MOTION:
                memcpy(&node.data.significantMotionData,
                    ptr + 2,
                    sizeof(node.data.
                    significantMotionData));

                memcpy(&LAST_SENSOR_NODE(sensorId),
                    &node.data.significantMotionData,
                    sizeof(node.data.significantMotionData));

                relay_node.significantMotionData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.significantMotionData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.significantMotionData.timeStamp);

                relay_node.significantMotionData.significantMotionDetected =
                    node.data.significantMotionData.significantMotionDetected;

                ptr += sizeof(node.data.significantMotionData) + 2;

#if 1
                dev_err(&sensor->client->dev,
                    "[OSP]: sensor %d timeStamp %llu significantMotionDetected %d ",
                    relay_node.significantMotionData.sensorId,
                    relay_node.significantMotionData.timeStamp,
                    relay_node.significantMotionData.significantMotionDetected);
#endif
                sensor_relay_write(&SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);

                break;

            case SENSOR_ORIENTATION:
                memcpy(&node.data.orientationData,
                    ptr + 2,
                    sizeof(node.data.orientationData));

                memcpy(&LAST_QUATERNION_NODE(sensorId),
                    &node.data.orientationData,
                    sizeof(node.data.orientationData));

                relay_node.orientationData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.orientationData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.orientationData.timeStamp);

                relay_node.orientationData.Data[0] = node.data.orientationData.Data[0];
                relay_node.orientationData.Data[1] = node.data.orientationData.Data[1];
                relay_node.orientationData.Data[2] = node.data.orientationData.Data[2];

#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-4.4x%-8.8x",
                    __func__,
                    sensorId,
                    node.orientationData.timeStamp.timeStamp40,
                    node.orientationData.timeStamp.timeStamp32);
#endif
                ptr += sizeof(node.data.orientationData) + 2;
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%16.16llx  x: %4.4x Y: %4.4x Z: %4.4x w: %4.4x e: %4.4x ",
                    __func__,
                    relay_node.orientationData.sensorId,
                    relay_node.orientationData.timeStamp,
                    relay_node.orientationData.Data[0],
                    relay_node.orientationData.Data[1],
                    relay_node.orientationData.Data[2]);
#endif
                sensor_relay_write(
                    &SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);
                break;

            case SENSOR_ROTATION_VECTOR:
            case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            case SENSOR_GAME_ROTATION_VECTOR:
                memcpy(&node.data.quaternionData,
                    ptr + 2,
                    sizeof(node.data.quaternionData));

                memcpy(&LAST_QUATERNION_NODE(sensorId),
                    &node.data.quaternionData,
                    sizeof(node.data.quaternionData));

                relay_node.quaternionData.sensorId =
                    SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId;

                relay_node.quaternionData.timeStamp =
                    TimeStamp40_to_timeStamp64(
                    node.data.quaternionData.timeStamp);

                relay_node.quaternionData.X = node.data.quaternionData.X;
                relay_node.quaternionData.Y = node.data.quaternionData.Y;
                relay_node.quaternionData.Z = node.data.quaternionData.Z;
                relay_node.quaternionData.W = node.data.quaternionData.W;
                relay_node.quaternionData.E_EST = node.data.quaternionData.E_EST;

#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%-4.4x%-8.8x",
                    __func__,
                    sensorId,
                    node.quaternionData.timeStamp.timeStamp40,
                    node.quaternionData.timeStamp.timeStamp32);
#endif
                ptr += sizeof(node.data.quaternionData) + 2;
#if 0
                dev_err(&sensor->client->dev,
                    "%s: sensor %d TimeStamp 0x%16.16llx  x: %4.4x Y: %4.4x Z: %4.4x w: %4.4x e: %4.4x ",
                    __func__,
                    relay_node.quaternionData.sensorId,
                    relay_node.quaternionData.timeStamp,
                    relay_node.quaternionData.X,
                    relay_node.quaternionData.Y,
                    relay_node.quaternionData.Z,
                    relay_node.quaternionData.W,
                    relay_node.quaternionData.E_EST);
#endif
                sensor_relay_write(
                    &SENSOR_DATA(sensorId).sensor_relay_cdev,
                    &relay_node);
                break;
            default:
                dev_err(&sensor->client->dev,
                    "%s: received invalid sensor id"
                    " 0x%x in broadcast data"
                    " offset 0x%x of size %d\n",
                    __func__,
                    sensorId, ptr - sensor->packet, length);

                for (ptr =  sensor->packet;
                    (ptr < (sensor->packet + length)) &&
                    (ptr < (sensor->packet + 0x40));
                ptr++) {

                    if (((ptr - sensor->packet) % 0x10) == 0)
                        printk("\n");

                    printk("0x%x ", *ptr);
                }
                break;
            }
        }
        sensor_relay_wakeup(&SENSOR_DATA(SENSOR_ACCELEROMETER_UNCALIBRATED).sensor_relay_cdev);
    }
    /*    dev_info(&sensor->client->dev, "osp_sh_work_func End"); */

    enable_irq(sensor->client->irq);
}



/*******************************************************************************
 * @fn      sensorhub_interrupt_thread
 *          Interrupt handler for Sensor Hub IRQ request
 *
 ******************************************************************************/
static irqreturn_t sensorhub_interrupt_thread(int irq, void *irq_data)
{
    struct sensorhub_sensor *sensor = (struct sensorhub_sensor *)irq_data;

    disable_irq_nosync(irq);

    schedule_work(&sensor->work);

    return IRQ_HANDLED;
}


/*******************************************************************************
 * @fn      osp_sh_set_sensor_delay
 *          Sends command over I2C to set the sensor output data rate
 *
 ******************************************************************************/
static int osp_sh_set_sensor_delay(
        struct sensorhub_sensor *sensor,
        SensorType_t sensorId,
        unsigned int delay) {
    int err = 0;

    struct ShSensorSetCmdHeader_16bits_param_t cmd;

    cmd.command = OSP_SH_SENSOR_SET_DELAY;
    cmd.sensorId = sensorId;
    cmd.param = delay;

    err = sensorhub_send_buf(sensor->client, (char *) &cmd, sizeof(cmd));

    if (err >= 0) {
        SENSOR_DATA(sensorId).delay = delay;

        dev_info(&sensor->client->dev, "osp_sh_set_sensor_delay  %ud  sensor %s\n",
            delay ,
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name);
    }
    return err;
}


/*******************************************************************************
 * @fn      osp_sh_set_relay_sensor_delay
 *          Sensor rate command handler on the relay side
 *
 ******************************************************************************/
static int osp_sh_set_relay_sensor_delay(
        struct sensor_relay_classdev *sensor_relay_cdev,
        unsigned int delay) {

    struct sensor_data *datas;
    struct sensorhub_sensor *sensor;

    datas = container_of(sensor_relay_cdev, struct sensor_data,
                         sensor_relay_cdev);

    sensor = container_of(datas, struct sensorhub_sensor,
                          sensor_data[sensor_relay_cdev->inputSensorId]);

    return osp_sh_set_sensor_delay(
        sensor,
        sensor_relay_cdev->inputSensorId,
        delay);
}

/*******************************************************************************
 * @fn      osp_sh_get_sensor_delay
 *          Sends command over I2C to read the sensor output data rate
 *
 ******************************************************************************/
static int osp_sh_get_sensor_delay(
        struct sensorhub_sensor *sensor,
        SensorType_t sensorId,
        unsigned int *delay) {

    int err = 0;

    struct ShCmdGetHeader_get_16bits_param_t data;

    err = sensorhub_sensor_read_buf(
        sensor->client,
        sensorId,
        OSP_SH_SENSOR_GET_DELAY,
        (char *) &data, sizeof(data));

    if (err >= 0) {
        *delay = data.param;

        dev_info(&sensor->client->dev, "osp_sh_get_sensor_delay  %ud  sensor %s\n",
            data.param ,
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name);
    }

    SENSOR_DATA(sensorId).delay = data.param;
    return err;
}

/*******************************************************************************
 * @fn      osp_sh_get_relay_sensor_delay
 *          Sensor rate command handler on the relay side for getting rate
 *
 ******************************************************************************/
static int osp_sh_get_relay_sensor_delay(
        struct sensor_relay_classdev *sensor_relay_cdev,
        unsigned int *delay) {

    struct sensor_data *datas;
    struct sensorhub_sensor *sensor;

    datas = container_of(sensor_relay_cdev, struct sensor_data,
                         sensor_relay_cdev);
    sensor = container_of(datas, struct sensorhub_sensor,
                          sensor_data[sensor_relay_cdev->inputSensorId]);

    return osp_sh_get_sensor_delay(
        sensor,
        sensor_relay_cdev->inputSensorId,
        delay);
}


/*******************************************************************************
 * @fn      osp_sh_set_sensor_enabled
 *          Sends command over I2C to enable/disable the sensors
 *
 ******************************************************************************/
static int osp_sh_set_sensor_enabled(
        struct sensorhub_sensor *sensor,
        SensorType_t sensorId,
        bool enable, bool suspended) {

    int err = 0;

    struct ShSensorSetCmdHeader_8bits_param_t cmd;

    dev_info(&sensor->client->dev, "osp_sh_set_sensor_enabled  %s  sensor %s\n",
        enable ? "Enable" : "Disable",
        SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name);

    cmd.command = OSP_SH_SENSOR_SET_ENABLE;
    cmd.sensorId = sensorId;
    cmd.param = enable ? 0xff : 0x00;
    sensorhub_send_buf(sensor->client, (u8 *)&cmd, sizeof(cmd));

    if (!suspended) {
        SENSOR_DATA(sensorId).enabled = enable;
    }
    return err;
}


/*******************************************************************************
 * @fn      osp_sh_set_relay_sensor_enabled
 *          Sensor enable command handler on the relay side for enable/disable
 *
 ******************************************************************************/
static int osp_sh_set_relay_sensor_enabled(
        struct sensor_relay_classdev *sensor_relay_cdev,
        bool enable) {

    struct sensor_data *datas;
    struct sensorhub_sensor *sensor;


    datas = container_of(sensor_relay_cdev, struct sensor_data,
                         sensor_relay_cdev);
    sensor = container_of(datas, struct sensorhub_sensor,
                          sensor_data[sensor_relay_cdev->inputSensorId]);

    dev_info(&sensor->client->dev, "osp_sh_set_relay_sensor_enabled  %s  sensor %s\n",
        enable ? "Enable" : "Disable", sensor_relay_cdev->sensor_name);

    return osp_sh_set_sensor_enabled(
        sensor,
        sensor_relay_cdev->inputSensorId,
        enable,
        false);
 }


/*******************************************************************************
 * @fn      osp_sh_get_sensor_enabled
 *          returns the current state of the sensor (enabled/disabled)
 *
 ******************************************************************************/
static int osp_sh_get_sensor_enabled(
        struct sensorhub_sensor *sensor,
        SensorType_t sensorId,
        bool * enable) {

    int err = 0;

    struct ShCmdGetHeader_get_8bits_param_t data;

    err = sensorhub_sensor_read_buf(
        sensor->client,
        sensorId,
        OSP_SH_SENSOR_GET_ENABLE,
        (char *) &data, sizeof(data));



    if (err >= 0) {
        *enable = data.param;
        SENSOR_DATA(sensorId).enabled = data.param;
        dev_info(&sensor->client->dev, "osp_sh_get_sensor_enabled  %s  sensor %s\n",
            data.param ? "Enabled" : "Disabled",
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name);
    }
    return err;
}

/*******************************************************************************
 * @fn      osp_sh_get_relay_sensor_enabled
 *          Relay side handler for getting the current state of the sensor
 *          (enabled/disabled)
 *
 ******************************************************************************/
static int osp_sh_get_relay_sensor_enabled(
        struct sensor_relay_classdev *sensor_relay_cdev,
        bool * enable) {

    struct sensor_data *datas;
    struct sensorhub_sensor *sensor;

    datas = container_of(sensor_relay_cdev, struct sensor_data,
                         sensor_relay_cdev);
    sensor = container_of(datas, struct sensorhub_sensor,
                          sensor_data[sensor_relay_cdev->inputSensorId]);

    return osp_sh_get_sensor_enabled(
        sensor,
        sensor_relay_cdev->inputSensorId,
        enable);
}


/*******************************************************************************
 * @fn      osp_sh_relay_init
 *          Initialize the relay driver
 *
 ******************************************************************************/
static int osp_sh_relay_init(
        struct sensorhub_sensor *sensor) {
    int err = 0;

    SensorType_t sensorId;

    for (sensorId = OSP_SH_SENSOR_ID_FIRST;
         sensorId < OSP_SH_SENSOR_ID_COUNT; sensorId++) {

        /* although this value is the index to this entry, storing it
         simplify the sensor relay class callbacks. do not remove */
        SENSOR_DATA(sensorId).sensor_relay_cdev.inputSensorId = sensorId;

        switch (sensorId) {
        case SENSOR_ACCELEROMETER_UNCALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "rawAccel";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_ACCELEROMETER_UNCALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 20;
            break;

       case SENSOR_ACCELEROMETER_CALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "calAccel";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_ACCELEROMETER_CALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 20;
            break;

        case SENSOR_GYROSCOPE_UNCALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "rawGyro";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_GYROSCOPE_UNCALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 10;
            break;

        case SENSOR_GYROSCOPE_CALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "calGyro";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_GYROSCOPE_CALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 10;
            break;

        case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "rawMag";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_MAGNETIC_FIELD_CALIBRATED:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "calMag";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_MAGNETIC_FIELD_CALIBRATED;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_ORIENTATION:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "orientation";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_ORIENTATION;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_CONTEXT_CHANGE_DETECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "change_detector";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_CONTEXT_CHANGE_DETECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_STEP_SEGMENT_DETECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "step_segment";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_STEP_SEGMENT_DETECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;
        case SENSOR_STEP_DETECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "step_detect";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_STEP_DETECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_STEP_COUNTER:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "step_counter";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_STEP_COUNTER;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_CONTEXT_DEVICE_MOTION:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "significant_motion";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_CONTEXT_DEVICE_MOTION;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_ROTATION_VECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "rotation_vector";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_ROTATION_VECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "geomagnetic_rotation_vector";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_GAME_ROTATION_VECTOR:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "game_rotation_vector";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_GAME_ROTATION_VECTOR;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_LINEAR_ACCELERATION:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "linear_acceleration";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_LINEAR_ACCELERATION;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        case SENSOR_GRAVITY:
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name =
                    "gravity";
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensorId =
                    SENSOR_GRAVITY;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay = 50;
            break;

        default:    /* never used */
            SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name = NULL;
            break;
        }

        if (SENSOR_DATA(sensorId).sensor_relay_cdev.sensor_name != NULL) {
            SENSOR_DATA(sensorId).sensor_relay_cdev.enable = false;
            SENSOR_DATA(sensorId).sensor_relay_cdev.flags = 0;
            SENSOR_DATA(sensorId).sensor_relay_cdev.enable_set =
                    osp_sh_set_relay_sensor_enabled;
            SENSOR_DATA(sensorId).sensor_relay_cdev.enable_get =
                    osp_sh_get_relay_sensor_enabled;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay_set =
                    osp_sh_set_relay_sensor_delay;
            SENSOR_DATA(sensorId).sensor_relay_cdev.delay_get =
                    osp_sh_get_relay_sensor_delay;
            err = sensor_relay_device_register(&sensor->client->dev,
                                               &SENSOR_DATA(sensorId).sensor_relay_cdev);

        }
    }
    pr_debug("[OSP]: osp_sh_relay_init: %d", err);
    return err;
}


/*******************************************************************************
 * @fn      sensorhub_probe
 *          The I2C layer calls us when it believes a sensor is present at this
 *          address. Probe to see if this is correct and to validate the device.
 *
 ******************************************************************************/
static int __devinit sensorhub_probe(struct i2c_client *client,
                                  const struct i2c_device_id *id)
{
    struct sensorhub_sensor *sensor = NULL;
    struct ShCmdGetHeader_get_8bits_param_t devId;
    struct ShCmdGetHeader_get_16bits_param_t devVersion;
    int err;
    SensorType_t sensorId;
    struct ShHubCmdHeader_t cmd;

    sensor = kzalloc(sizeof(struct sensorhub_sensor), GFP_KERNEL);
    if (!sensor) {
        err = -ENOMEM;
        goto err_free_mem;
    }
    sensor->client = client;
    sensor->dev = &client->dev;

    mutex_init(&sensor->mutex);


    i2c_set_clientdata(client, sensor);

    err = osp_sh_relay_init(sensor);
    if (err)
        goto err_free_relay;

    cmd.command = OSP_SH_RESET;
    sensorhub_send_buf(sensor->client, (u8 *)&cmd, sizeof(cmd));

    err = sensorhub_read_buf(
        client,
        OSP_SH_GET_WHO_AM_I,
        (u8 *)&devId,
        sizeof(devId));

    if (err < 0) {
        dev_err(&client->dev, "failed to detect device\n");
        err = -ENXIO;
        goto err_free_mem;
    }
    err = sensorhub_read_buf(
        client,
        OSP_SH_GET_VERSION,
        (u8 *)&devVersion,
        sizeof(devVersion));

    if (err < 0) {
        dev_err(&client->dev, "failed to detect device\n");
        err = -ENXIO;
        goto err_free_mem;
    }

    dev_info(
        &client->dev,
        "chip id  0x%x   version 0x%x\n",
        devId.param, devVersion.param);


    pm_runtime_set_active(&client->dev);

    INIT_WORK(&sensor->work, osp_sh_work_func);

    sensor->interruptsEnabled = 0;

    err = request_threaded_irq(client->irq,
                               NULL, sensorhub_interrupt_thread,
                               IRQF_TRIGGER_FALLING,
                               "sensorhub_int", sensor);
    if (err) {
        dev_err(&client->dev,
                "can't get IRQ %d, error %d\n", client->irq, err);
        goto err_pm_set_suspended;
    }

    //    disable_irq(client->irq);

    pm_runtime_enable(&client->dev);

    pm_runtime_set_autosuspend_delay(&client->dev, OSP_SH_SUSPEND_DELAY);

    pr_debug("Sensor Hub driver init successful!");
    return 0;

err_pm_set_suspended:
    pm_runtime_set_suspended(&client->dev);
err_free_relay:
    for (sensorId = OSP_SH_SENSOR_ID_FIRST;
         sensorId < OSP_SH_SENSOR_ID_COUNT; sensorId++)
        sensor_relay_device_unregister(&SENSOR_DATA(sensorId).sensor_relay_cdev);
err_free_mem:
    if (sensor) kfree(sensor);
    return err;
}


/*******************************************************************************
 * @fn      sensorhub_remove
 *          Our device is going away, clean up the resources.
 *
 ******************************************************************************/
static int __devexit sensorhub_remove(struct i2c_client *client)
{
    SensorType_t sensorId;
    struct sensorhub_sensor *sensor = i2c_get_clientdata(client);

    pm_runtime_disable(&client->dev);
    pm_runtime_set_suspended(&client->dev);

    free_irq(client->irq, sensor);

    for (sensorId = OSP_SH_SENSOR_ID_FIRST; sensorId < OSP_SH_SENSOR_ID_COUNT; sensorId++)
        sensor_relay_device_unregister(&SENSOR_DATA(sensorId).sensor_relay_cdev);
    kfree(sensor);

    return 0;
}

#ifdef CONFIG_PM
/*******************************************************************************
 * @fn      sensorhub_suspend
 *          Put the device into low power mode before we suspend the machine.
 *
 ******************************************************************************/
static int sensorhub_suspend(struct device *dev)
{
    SensorType_t sensorId;

    struct i2c_client *client = to_i2c_client(dev);

    struct sensorhub_sensor *sensor = i2c_get_clientdata(client);

    for (sensorId = OSP_SH_SENSOR_ID_FIRST;
        sensorId < OSP_SH_SENSOR_ID_COUNT;
        sensorId++) {
        if (SENSOR_DATA(sensorId).enabled) {
            osp_sh_set_sensor_enabled(sensor, sensorId, false, true);
        }
    }
    return 0;
}


/*******************************************************************************
 * @fn      sensorhub_resume
 *          Put the device into powered mode on resume.
 *
 ******************************************************************************/
static int sensorhub_resume(struct device *dev)
{
    SensorType_t sensorId;

    struct i2c_client *client = to_i2c_client(dev);

    struct sensorhub_sensor *sensor = i2c_get_clientdata(client);

    for (sensorId = OSP_SH_SENSOR_ID_FIRST;
        sensorId < OSP_SH_SENSOR_ID_COUNT;
        sensorId++) {
        if (SENSOR_DATA(sensorId).enabled) {
            osp_sh_set_sensor_enabled(sensor, sensorId, true, true);
        }
    }

    return 0;
}
#endif

/* Standard Linux Kernel Module declaration */
static UNIVERSAL_DEV_PM_OPS(sensorhub_pm, sensorhub_suspend, sensorhub_resume, NULL);

static const struct i2c_device_id sensorhub_ids[] = {
    { "spihub", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sensorhub_ids);

static struct i2c_driver sensorhub_i2c_driver = {
    .driver = {
        .name   = "spihub",
        .owner  = THIS_MODULE,
        .pm = &sensorhub_pm,
    },
    .probe      = sensorhub_probe,
    .remove     = __devexit_p(sensorhub_remove),
    .id_table   = sensorhub_ids,
};

static int __init sensorhub_init(void)
{
    return i2c_add_driver(&sensorhub_i2c_driver);
}
module_init(sensorhub_init);

static void __exit sensorhub_exit(void)
{
    i2c_del_driver(&sensorhub_i2c_driver);
}
module_exit(sensorhub_exit);

MODULE_AUTHOR("Shmuel Ungerfeld, Rajiv Verma, Hunyue Yau");
MODULE_DESCRIPTION("Sensor Platforms Inc., Sensor hub driver");
MODULE_LICENSE("GPL");
