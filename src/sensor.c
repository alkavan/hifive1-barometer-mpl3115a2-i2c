#include "sensor.h"

#include <drivers/i2c.h>

/*
 * local functions *****************************************************************************************************
 */
static void display_i2c_write(int opcode, const char * reg_name, uint8_t reg_addr, uint8_t value);

/*
 * public functions ****************************************************************************************************
 */
bool activate_mpl3115a2_device(const struct device *i2c_dev, uint32_t i2c_cfg) {
    if (i2c_dev == NULL || ! device_is_ready(i2c_dev))
    {
        printk("[i2c] could not get device\n");
        return false;
    } else {
        if (i2c_configure(i2c_dev, i2c_cfg)) {
            printk("[i2c] config failed\n");
            return false;
        } else {
            printk("[i2c] device ready\n");
        }
    }

    return true;
}

bool setup_mpl3115a2_device(const struct device *i2c_dev) {
    int opcode;

    /* Set to Altimeter with an OSR = 128 */
    opcode = i2c_reg_write_byte(i2c_dev, MPL3115A2_8BIT_WRITE, MPL3115A2_CTRL_REG1, 0xB8);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, 0xB8);
    if (opcode) { return false; }

    /* Enable Data Flags in PT_DATA_CFG */
    opcode = i2c_reg_write_byte(i2c_dev, MPL3115A2_8BIT_WRITE, MPL3115A2_PT_DATA_CFG, 0x07);
    display_i2c_write(opcode, "PT_DATA_CFG", MPL3115A2_PT_DATA_CFG, 0x07);
    if (opcode) { return false; }

    /* Set Active */
    opcode = i2c_reg_write_byte(i2c_dev, MPL3115A2_8BIT_WRITE, MPL3115A2_CTRL_REG1, 0xB9);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, 0xB9);
    if (opcode) { return false; }

    return true;
}

int read_sensor_register(const struct device *i2c_dev, const uint8_t reg_addr, uint8_t * value) {
    int ret = i2c_reg_read_byte(i2c_dev, MPL3115A2_8BIT_READ, reg_addr, value);

    if (ret) {
        printk("[i2c] failed reading register: %x\n", reg_addr);
    }

    return ret;
}

void reset_sensor_errors(int32_t * sensor_error, size_t size) {
    for(int i = 0; i < size; i++) {
        sensor_error[i] = 0;
    }
}

uint32_t sum_sensor_errors(const int32_t * sensor_error, size_t size) {
    uint32_t sum = 0;

    for(int i = 0; i < size; i++) {
        sum += sensor_error[i];
    }

    return sum;
}

// local functions *****************************************************************************************************
static void display_i2c_write(const int opcode, const char *reg_name, const uint8_t reg_addr, const uint8_t value) {
    if (opcode) {
        printk("[i2c] failed writing to %s at %02x\n", reg_name, reg_addr);
    } else {
        printk("[i2c] wrote %02x to %s at %02x\n", value, reg_name, reg_addr);
    }
}

uint32_t sensor_get_config()  {
    return I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;
}
