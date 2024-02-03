#include "display.h"

#include <drivers/spi.h>

bool activate_display_device(const struct device *spi_dev) {
    if (spi_dev == NULL || ! device_is_ready(spi_dev))
    {
        printk("[spi] could not get device\n");
        return false;
    } else {
        printk("[spi] device ready\n");
    }

    return true;
}
