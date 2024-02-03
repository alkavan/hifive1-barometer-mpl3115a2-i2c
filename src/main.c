#include <stdio.h>
#include <kernel.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "leds.h"
#include "sensor.h"
#include "display.h"

LOG_MODULE_REGISTER(logger, CONFIG_LOG_DEFAULT_LEVEL);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS (1000)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#define I2C_NODE DT_NODELABEL(i2c0)
#define SPI_NODE DT_NODELABEL(spi1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

/**
 * LEDs
 */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

#define ERROR_BUFFER_SIZE 6
#define DATA_BUFFER_SIZE 8

int main(void)
{
    /**
     * I2C config
     */
     const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
     const uint32_t i2c_cfg = sensor_get_config();

    /**
     * SPI config
     */
    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

    int opcode;

    printk("[sys] initialing ...\n");

    // green
    bool led0_ok = activate_led_device(&led0, 0);

    // blue
    bool led1_ok = activate_led_device(&led1, 0);

    // red
    bool led2_ok = activate_led_device(&led2, 0);

    // i2c
    bool i2c_ok = activate_mpl3115a2_device(i2c_dev, i2c_cfg)
            && setup_mpl3115a2_device(i2c_dev);

    // spi
    bool spi_ok = activate_display_device(spi_dev);

    // sensor buffers
    int32_t sensor_error[ERROR_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0};
    uint8_t sensor_buffer[DATA_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

    uint64_t device_cycle = 0;

    while (1) {
        device_cycle++;
        printk("[> ---- (%ums) cycle #%09llu -------------------------------- <]\n", SLEEP_TIME_MS, device_cycle);

        if(i2c_ok) {
            /* Read STATUS Register */
            sensor_error[0] = read_sensor_register(i2c_dev, MPL3115A2_8BIT_READ, &sensor_buffer[0]);

            if( ! sensor_error[0]) {
                printk("[i2c] sensor status: %02x\n", sensor_buffer[0]);

                if((sensor_buffer[0] & 0x08) > 0 ) {
                    printk("[i2c] data ready, reading sensors ... (%ums)\n", SLEEP_TIME_MS);

                    reset_sensor_errors(sensor_error, ERROR_BUFFER_SIZE);

                    sensor_error[1] = read_sensor_register(i2c_dev, MPL3115A2_REGISTER_PRESSURE_MSB, &sensor_buffer[1]);
                    sensor_error[2] = read_sensor_register(i2c_dev, MPL3115A2_REGISTER_PRESSURE_CSB, &sensor_buffer[2]);
                    sensor_error[3] = read_sensor_register(i2c_dev, MPL3115A2_REGISTER_PRESSURE_LSB, &sensor_buffer[3]);
                    sensor_error[4] = read_sensor_register(i2c_dev, MPL3115A2_REGISTER_TEMP_MSB, &sensor_buffer[4]);
                    sensor_error[5] = read_sensor_register(i2c_dev, MPL3115A2_REGISTER_TEMP_LSB, &sensor_buffer[5]);

                    uint32_t error_sum = sum_sensor_errors(sensor_error, ERROR_BUFFER_SIZE);

                    if(error_sum != 0) {
                        printk("[i2c] error reading from sensors ( %d %d %d %d %d %d )\n",
                               sensor_error[0], sensor_error[1], sensor_error[2],
                               sensor_error[3], sensor_error[4], sensor_error[5]);
                    } else {
                        printk("[i2c] pressure: ( %03u %03u %03u ) temperature: ( %03u, %03u )\n",
                               sensor_buffer[1], sensor_buffer[2], sensor_buffer[3],
                               sensor_buffer[4], sensor_buffer[5]);
                    }
                }
            }
        }

        if(spi_ok) {
            // @todo: implement display via spi.
        }

        // @todo: show device status using rgb led.
        if (led0_ok) {
//            ret = gpio_pin_set_dt(&led0, 0);
            opcode = gpio_pin_toggle_dt(&led0);
            if (opcode < 0) {
                return 0;
            }
        }
//
        if (led1_ok) {
            opcode = gpio_pin_set_dt(&led1, 0);
//            ret = gpio_pin_toggle_dt(&led1);
            if (opcode < 0) {
                return 0;
            }
        }

        if (led2_ok) {
            opcode = gpio_pin_set_dt(&led2, 0);
//            ret = gpio_pin_toggle_dt(&led2);
            if (opcode < 0) {
                return 0;
            }
        }

        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
