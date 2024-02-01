#include <stdio.h>
#include <kernel.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/spi.h>
#include <logging/log.h>

// @todo: move from main.c
bool activate_device(const struct gpio_dt_spec * device_spec) {
    int ret;

    if ( ! device_is_ready(device_spec->port)) {
        return false;
    }

    ret = gpio_pin_configure_dt(device_spec, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return false;
    }

    return true;
}

bool activate_i2c_device(const struct device *i2c_dev, uint32_t i2c_cfg) {
    if (i2c_dev == NULL || ! device_is_ready(i2c_dev))
    {
        printk("could not get I2C device\n");
        return false;
    } else {
        if (i2c_configure(i2c_dev, i2c_cfg)) {
            printk("I2C config failed\n");
            return false;
        } else {
            printk("I2C device ready\n");
        }
    }

    return true;
}

bool activate_spi_device(const struct device *spi_dev) {
    if (spi_dev == NULL || ! device_is_ready(spi_dev))
    {
        printk("could not get SPI device");
        return false;
    } else {
        printk("SPI device ready");
    }

//    LOG_DBG("spi_cs.bus = %p", spi_dev.bus);
//    LOG_DBG("spi_cs.config.cs->gpio.port = %p", spi_cs.config.cs->gpio.port);
//    LOG_DBG("spi_cs.config.cs->gpio.pin = %u", spi_cs.config.cs->gpio.pin);
    return true;
}

void display_led_status(const bool * ok, uint8_t index) {
    printf("led%u: %s\n", index, ok ? "ok" : "fail");
}

LOG_MODULE_REGISTER(logger, CONFIG_LOG_DEFAULT_LEVEL);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 34

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#define I2C_NODE DT_NODELABEL(i2c0)
#define SPI_NODE DT_NODELABEL(spi0)

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

K_HEAP_DEFINE(heap1, 1024);

uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;

int main(void)
{
    /**
     * I2C
     */
     const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
//     const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_GET(I2C_NODE1);

    /**
     * SPI
     */
//    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

    int ret;

    printf("\n");

    // green
    bool led0_ok = activate_device(&led0);
    display_led_status(&led0_ok, 0);

    // blue
    bool led1_ok = activate_device(&led1);
    display_led_status(&led1_ok, 1);

    // red
    bool led2_ok = activate_device(&led2);
    display_led_status(&led2_ok, 2);

    // i2c
    activate_i2c_device(i2c_dev, i2c_cfg);

    // spi
//    activate_spi_device(spi_dev);

    // try to allocate heap
//    k_timeout_t timeout = {100};
//    k_heap_init(&heap1, NULL, 1024);


//    uint8_t *i2c_buffer = NULL;
//    i2c_buffer = k_heap_alloc(&heap1, 1024, K_NO_WAIT);
//    printk("heap1 addr: %p\n", i2c_buffer);

    uint8_t i2c_buffer[1024] = {};

    while (1) {

        i2c_read(i2c_dev, i2c_buffer, 1, 0x02);
        printk("buffer[i2c]: %s\n", i2c_buffer);

        if (led0_ok) {
//            ret = gpio_pin_set_dt(&led0, 0);
            ret = gpio_pin_toggle_dt(&led0);
            if (ret < 0) {
                return 0;
            }
        }
//
        if (led1_ok) {
            ret = gpio_pin_set_dt(&led1, 0);
//            ret = gpio_pin_toggle_dt(&led1);
            if (ret < 0) {
                return 0;
            }
        }

        if (led2_ok) {
            ret = gpio_pin_set_dt(&led2, 0);
//            ret = gpio_pin_toggle_dt(&led2);
            if (ret < 0) {
                return 0;
            }
        }

        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
