#ifndef HIFIVE1_BAROMETER_MPL3115A2_I2C_DISPLAY_H
#define HIFIVE1_BAROMETER_MPL3115A2_I2C_DISPLAY_H

#include <kernel.h>
#include <device.h>

/**
 * @brief Activate display device.
 *
 * @param spi_dev
 * @return
 */
bool activate_display_device(const struct device *spi_dev);

#endif //HIFIVE1_BAROMETER_MPL3115A2_I2C_DISPLAY_H
