#ifndef HIFIVE1_BAROMETER_MPL3115A2_I2C_LEDS_H
#define HIFIVE1_BAROMETER_MPL3115A2_I2C_LEDS_H

#include <kernel.h>
#include <drivers/gpio.h>

/**
 * @brief Activate led device (default on).
 *
 * @param device_spec
 * @param index
 * @return
 */
bool activate_led_device(const struct gpio_dt_spec * device_spec, uint8_t index);

#endif //HIFIVE1_BAROMETER_MPL3115A2_I2C_LEDS_H
