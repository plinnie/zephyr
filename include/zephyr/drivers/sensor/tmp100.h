#ifndef TMP100_H
#define TMP100_H

/*
    Example setting the resulution to 12 bits:
    struct sensor_value conf;
    sensor_attr_get(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_CONFIGURATION, &conf);
    conf.val1 = TMP100_CONFIG_TEMP_RES_12BITS; 
    sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_CONFIGURATION, &conf);   
*/

/* Configuration register bits (see tmp100 datasheet for details) */
#define TMP100_CONFIG_SD                1
#define TMP100_CONFIG_TM                (1 << 1)
#define TMP100_CONFIG_POL               (1 << 2)
#define TMP100_CONFIG_FQ_1              0
#define TMP100_CONFIG_FQ_2              (1 << 3)
#define TMP100_CONFIG_FQ_3              (1 << 4)
#define TMP100_CONFIG_FQ_4              ((1 << 3) | (1 << 4))
#define TMP100_CONFIG_TEMP_RES_9BITS    0
#define TMP100_CONFIG_TEMP_RES_10BITS   (1 << 5)
#define TMP100_CONFIG_TEMP_RES_11BITS   (1 << 6)
#define TMP100_CONFIG_TEMP_RES_12BITS   ((1 << 5) | (1 << 6))
#define TMP100_CONFIG_SO                (1 << 7)

#endif