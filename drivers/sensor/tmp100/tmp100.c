/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp100

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "tmp100.h"

LOG_MODULE_REGISTER(TMP100, CONFIG_SENSOR_LOG_LEVEL);

static int tmp100_reg_read_uint16(const struct i2c_dt_spec *i2c, uint8_t reg, unsigned len, uint16_t *val)
{
	uint8_t data[2];

	__ASSERT_NO_MSG(len > 0 && len <= sizeof(data));

	if (i2c_burst_read_dt(i2c, reg, data, len) < 0) {
		LOG_ERR("I2C read failed");
		return -EIO;
	}

	if(len == 2)
		*val = (data[0] * 256) + data[1];
	else
		*val = data[0];

	return 0;
}

static int tmp100_reg_write_uint8(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *val)
{
	uint8_t tx_buf[] = {reg, *val};
	return i2c_write_dt(i2c, tx_buf, sizeof(tx_buf));
}


static int tmp100_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tmp100_data *drv_data = dev->data;
	const struct tmp100_config *cfg = dev->config;
	uint16_t val;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL)
		return -ENOTSUP;

	if (tmp100_reg_read_uint16(&cfg->i2c, TMP100_REG_TEMP, 2, &val) < 0)
		return -EIO;

	drv_data->sample = arithmetic_shift_right(val, 4);

	return 0;
}

static int tmp100_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct tmp100_data *drv_data = dev->data;
	int32_t uval;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	uval = (int32_t)drv_data->sample;
	val->val1 = uval / 16;
	val->val2 = ((uval % 16) * 1e6) / 16;

	return 0;
}

static int tmp100_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_AMBIENT_TEMP || attr != SENSOR_ATTR_CONFIGURATION)
		return -ENOTSUP;

	const struct tmp100_config *cfg = dev->config;
	uint8_t rval = val->val1;
	if (tmp100_reg_write_uint8(&cfg->i2c, TMP100_REG_CONFIG, &rval) < 0)
		return -EIO;

	return 0;
}

static int tmp100_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_AMBIENT_TEMP || attr != SENSOR_ATTR_CONFIGURATION)
		return -ENOTSUP;

	const struct tmp100_config *cfg = dev->config;
	uint16_t rval;
	if (tmp100_reg_read_uint16(&cfg->i2c, TMP100_REG_CONFIG, 1, &rval) < 0)
		return -EIO;

	val->val1 = rval;
	return 0;
}

static const struct sensor_driver_api tmp100_driver_api = {
	.sample_fetch = tmp100_sample_fetch,
	.channel_get = tmp100_channel_get,
	.attr_set = tmp100_attr_set,
	.attr_get = tmp100_attr_get
};

int tmp100_init(const struct device *dev)
{
	const struct tmp100_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus device is not ready");
		return -ENODEV;
	}

	uint8_t rval = TMP100_RES_BITS(cfg->resolution);
	if (tmp100_reg_write_uint8(&cfg->i2c, TMP100_REG_CONFIG, &rval) < 0)
		return -EIO;

	LOG_INF("%s driver init", dev->name);
	return 0;
}

#define TMP100_DEFINE(inst)									\
	static struct tmp100_data tmp100_data_##inst;						\
												\
	static const struct tmp100_config tmp100_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		.resolution = DT_PROP(DT_INST(inst,DT_DRV_COMPAT),resolution) \
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tmp100_init, NULL,					\
			      &tmp100_data_##inst, &tmp100_config_##inst, POST_KERNEL,		\
			      CONFIG_SENSOR_INIT_PRIORITY, &tmp100_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(TMP100_DEFINE)
