/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TMP100_TMP100_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP100_TMP100_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#define TMP100_REG_TEMP			0x00
#define TMP100_REG_CONFIG		0x01
#define TMP100_REG_TLOW			0x02
#define TMP100_REG_THIGH		0x03

#define TMP100_RES_BITS(res)	((res - 9) << 5) // r5,r6 of config register determinate the resolution, 9-12 bits 

struct tmp100_config {
	struct i2c_dt_spec i2c;
	int resolution;
};

struct tmp100_data {
	int16_t sample;
};


#endif /* _SENSOR_TMP100_ */
