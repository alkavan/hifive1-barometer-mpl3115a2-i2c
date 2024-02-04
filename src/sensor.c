#include "sensor.h"
#include "config.h"

#include <drivers/i2c.h>

/*
 * local functions *****************************************************************************************************
 */
static void display_i2c_write(int opcode, const char * reg_name, uint8_t reg_addr, uint8_t value);

/*
 * public functions ****************************************************************************************************
 */
bool activate_mpl3115a2_device(const struct device * i2c_dev, uint32_t i2c_cfg) {
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

bool setup_mpl3115a2_device(const struct device * i2c_dev) {
    int opcode;

    /* Set to Altimeter with an OSR = 128 */
    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_OS128);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_OS128);
    if (opcode) { return false; }

    /* Enable Data Flags in PT_DATA_CFG */
    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_PT_DATA_CFG, 0x07);
    display_i2c_write(opcode, "PT_DATA_CFG", MPL3115A2_PT_DATA_CFG, 0x07);
    if (opcode) { return false; }

    /* Set Active */
    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_CTRL_REG1, 0xB9);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, 0xB9);
    if (opcode) { return false; }

    return true;
}

bool do_mpl3115a2_cycle(const struct device * i2c_dev,
        int32_t * sensor_error,
        uint8_t * sensor_buffer,
        uint32_t sleep_ms
) {
    /* Read STATUS Register */
    sensor_error[0] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_STATUS, &sensor_buffer[0]);

    if( ! sensor_error[0]) {
//        printk("[i2c] sensor status: %02x\n", sensor_buffer[0]);

        if(sensor_buffer[0] & MPL3115A2_STATUS_DATA_RDY_BITMASK) {
//            printk("[i2c] data ready, reading sensors ... (sleep: %ums)\n", sleep_ms);

            sensor_reset_errors(sensor_error, ERROR_BUFFER_SIZE);

            sensor_error[1] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_MSB, &sensor_buffer[1]);
            sensor_error[2] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_CSB, &sensor_buffer[2]);
            sensor_error[3] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_LSB, &sensor_buffer[3]);
            sensor_error[4] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_TEMP_MSB, &sensor_buffer[4]);
            sensor_error[5] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_TEMP_LSB, &sensor_buffer[5]);

            uint32_t error_sum = sensor_sum_errors(sensor_error, ERROR_BUFFER_SIZE);

            if(error_sum != 0) {
                printk("[i2c] error reading from sensors ( %d %d %d %d %d %d )\n",
                       sensor_error[0], sensor_error[1], sensor_error[2],
                       sensor_error[3], sensor_error[4], sensor_error[5]);
            } else {
                uint32_t pressure = ((uint32_t)sensor_buffer[1] << 24) | ((uint32_t)sensor_buffer[2] << 16) | ((uint32_t)sensor_buffer[3] << 8) | 0b00000000;
                uint32_t temperature = ((uint32_t)sensor_buffer[4] << 24) | ((uint32_t)sensor_buffer[5] << 16) | ((uint32_t)0b00000000 << 8) | 0b00000000;
                printk("[i2c] pressure: ( %010u ) temperature: ( %010u )\n", pressure, temperature);
            }
        }
    }

    return true;
}

uint32_t sensor_get_config()  {
    return I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;
}

int sensor_read_8bit(const struct device * i2c_dev, const uint8_t reg_addr, uint8_t * value) {
    int opcode = i2c_reg_read_byte(i2c_dev, MPL3115A2_8BIT_WRITE, reg_addr, value);

    if (opcode) {
        printk("[i2c] failed reading register: %x (dev_addr: %02x)\n", reg_addr, MPL3115A2_8BIT_WRITE);
    }

    return opcode;
}

int sensor_write_8bit(const struct device * i2c_dev, const uint8_t reg_addr, uint8_t value) {
    int opcode = i2c_reg_write_byte(i2c_dev, MPL3115A2_8BIT_WRITE, reg_addr, value);

    if (opcode) {
        printk("[i2c] failed writing register: %02x (dev_addr: %02x)\n", reg_addr, MPL3115A2_8BIT_WRITE);
    }

    return opcode;
}

void sensor_reset_errors(int32_t * sensor_error, size_t size) {
    for(int i = 0; i < size; i++) {
        sensor_error[i] = 0;
    }
}

uint32_t sensor_sum_errors(const int32_t * sensor_error, size_t size) {
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
