#include <stdio.h>
#include <kernel.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "leds.h"
#include "sensor.h"
#include "display.h"
#include "config.h"

LOG_MODULE_REGISTER(logger, CONFIG_LOG_DEFAULT_LEVEL);

/* Devicetree node identifier for the LED aliases. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

/* Devicetree node identifier for the I2C alias. */
#define I2C_NODE DT_NODELABEL(i2c0)

/* Devicetree node identifier for the SPI alias. */
#define SPI_NODE DT_NODELABEL(spi1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

/**
 * Define specs for the LEDs:
 */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

int main(void)
{
    /**
     * I2C config
     */
     const struct device * i2c_dev = DEVICE_DT_GET(I2C_NODE);
     const uint32_t i2c_cfg = sensor_get_config();

    /**
     * SPI config
     */
    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

    int opcode;

    printk("[sys] initialing ...\n");

    // green LED
    bool led0_ok = activate_led_device(&led0, 0);

    // blue LED
    bool led1_ok = activate_led_device(&led1, 0);

    // red LED
    bool led2_ok = activate_led_device(&led2, 0);

    // I2C
    bool i2c_ok = activate_mpl3115a2_device(i2c_dev, i2c_cfg)
            && setup_mpl3115a2_device(i2c_dev);

    // SPI
    bool spi_ok = activate_display_device(spi_dev);

    // sensor buffers
    int32_t sensor_error[ERROR_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0};
    uint8_t sensor_buffer[DATA_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

    uint64_t device_cycle = 0;

    while (1) {
        device_cycle++;
        if(device_cycle % 10 == 0) {
            printk("[> ---- cycle #%09llu -------------------------------- <]\n", device_cycle);
        }

        if(i2c_ok) {
            do_mpl3115a2_cycle(i2c_dev, sensor_error, sensor_buffer, SLEEP_TIME_MS);
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
