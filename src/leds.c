#include "leds.h"

#include <device.h>
#include <drivers/gpio.h>

/*
 * local functions *****************************************************************************************************
 */
static void display_led_status(const int opcode, uint8_t index);

/*
 * public functions
 */
bool activate_led_device(const struct gpio_dt_spec * device_spec, uint8_t index) {
    int ret;

    if ( ! device_is_ready(device_spec->port)) {
        return false;
    }

    ret = gpio_pin_configure_dt(device_spec, GPIO_OUTPUT_ACTIVE);

    // print activation status
    display_led_status(ret, index);

    if (ret < 0) {
        return false;
    }

    return true;
}

/*
 * local functions *****************************************************************************************************
 */
static void display_led_status(const int opcode, uint8_t index) {
    printk("[led:%u] %s\n", index, (opcode == 0) ? "ok" : "fail");
}