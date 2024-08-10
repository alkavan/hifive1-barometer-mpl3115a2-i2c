#include "sensor.h"
#include "config.h"

#include <stdio.h>

#include <drivers/i2c.h>

/*
 * local functions *****************************************************************************************************
 */
static void display_i2c_write(int opcode, const char * reg_name, uint8_t reg_addr, uint8_t value);
static void float_to_string(float number, char *buffer, int decimal_places);

static int32_t read_pressure(const uint8_t *sensor_buffer, bool raw);
static float read_pressure_f(const uint8_t *sensor_buffer, bool raw);

static int32_t read_altitude(const uint8_t *sensor_buffer);
static float read_altitude_f(const uint8_t *sensor_buffer);

static uint16_t read_temperature(const uint8_t *sensor_buffer);
static float read_temperature_f(const uint8_t *sensor_buffer);

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

uint8_t operation_mode = MPL3115A2_CTRL_REG1_OS128
        /*| MPL3115A2_CTRL_REG1_RAW*/
        | MPL3115A2_CTRL_REG1_BAR
        | MPL3115A2_CTRL_REG1_OST
        ;

bool setup_mpl3115a2_device(const struct device * i2c_dev) {
    int opcode;

    /* Reset device */
//    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
//    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
//    if (opcode) { return false; }

    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_CTRL_REG1, operation_mode);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, operation_mode);
    if (opcode) { return false; }

    /* Enable Data Flags in PT_DATA_CFG */
    uint8_t config_flag = MPL3115A2_PT_DATA_CFG_TDEFE
            | MPL3115A2_PT_DATA_CFG_PDEFE
            | MPL3115A2_PT_DATA_CFG_DREM;

    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_PT_DATA_CFG, config_flag);
    display_i2c_write(opcode, "PT_DATA_CFG", MPL3115A2_PT_DATA_CFG, config_flag);
    if (opcode) { return false; }

    /* Set Active */
    opcode = sensor_write_8bit(i2c_dev, MPL3115A2_CTRL_REG1, operation_mode | MPL3115A2_CTRL_REG1_SBYB);
    display_i2c_write(opcode, "CTRL_REG1", MPL3115A2_CTRL_REG1, operation_mode | MPL3115A2_CTRL_REG1_SBYB);
    if (opcode) { return false; }

    return true;
}

bool do_mpl3115a2_cycle(
        const struct device * i2c_dev,
        int32_t * sensor_error,
        uint8_t * sensor_buffer,
        uint32_t sleep_ms
) {
    /* Read status register */
    sensor_error[0] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_STATUS, &sensor_buffer[0]);
//    printk("[i2c] sensor status: %02x\n", sensor_buffer[0]);

        /* Sensor data is ready */
    if(sensor_buffer[0] & MPL3115A2_STATUS_DATA_RDY_BITMASK) {
//        printk("[i2c] data ready, reading sensors ... (sleep: %ums)\n", sleep_ms);

        sensor_reset_errors(sensor_error, ERROR_BUFFER_SIZE);

        /* Read pressure */
        sensor_error[1] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_MSB, &sensor_buffer[1]);
        sensor_error[2] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_CSB, &sensor_buffer[2]);
        sensor_error[3] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_PRESSURE_LSB, &sensor_buffer[3]);

        /* Read temperature */
        sensor_error[4] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_TEMP_MSB, &sensor_buffer[4]);
        sensor_error[5] = sensor_read_8bit(i2c_dev, MPL3115A2_REGISTER_TEMP_LSB, &sensor_buffer[5]);

        uint32_t error_sum = sensor_sum_errors(sensor_error, ERROR_BUFFER_SIZE);

        if(error_sum != 0) {
            printk("[i2c] error reading from sensors ( %d %d %d %d %d %d )\n",
                   sensor_error[0], sensor_error[1], sensor_error[2],
                   sensor_error[3], sensor_error[4], sensor_error[5]);
        } else {
            float pressure_f = read_pressure_f(sensor_buffer, true);
            float temperature_f = read_temperature_f(sensor_buffer);

            char buffer1[32] = {'\0'};
            char buffer2[32] = {'\0'};

            float_to_string(pressure_f, buffer1, 3);
            float_to_string(temperature_f, buffer2, 3);

            printf("[i2c] pressure: ( %s hPa ) temperature: ( %s °C )\n", buffer1, buffer2);
        }

        k_sleep(Z_TIMEOUT_MS(sleep_ms));
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

static int32_t read_pressure(const uint8_t *sensor_buffer, bool raw)
{
    // Combine the MSB, CSB, and LSB into a 20-bit value
    uint32_t pressure_raw = ((uint32_t)sensor_buffer[1] << 16) |
                            ((uint32_t)sensor_buffer[2] << 8)  |
                            ((uint32_t)sensor_buffer[3]);

    int32_t pressure_pascals;

    if(raw) {
        pressure_pascals = pressure_raw;
    } else {
        pressure_pascals = pressure_raw >> 6;  // Shift right by 6 to remove fractional bits
    }

    return pressure_pascals;
}

static float read_pressure_f(const uint8_t *sensor_buffer, bool raw) {
    return (float)read_pressure(sensor_buffer, raw) / (float)6400.0;
}

static int32_t read_altitude(const uint8_t *sensor_buffer) {
    int32_t alt = ((uint32_t)sensor_buffer[0] << 24)
                   | ((uint32_t)sensor_buffer[1] << 16)
                   | ((uint32_t)sensor_buffer[2] << 8);

    return alt;
}

static float read_altitude_f(const uint8_t *sensor_buffer)
{
    return (float)read_altitude(sensor_buffer) / (float)65536.0;
}

static uint16_t read_temperature(const uint8_t *sensor_buffer)
{
    return ( (uint16_t)sensor_buffer[4] << 8 ) | ( (uint16_t)sensor_buffer[5] );
}

static float read_temperature_f(const uint8_t *sensor_buffer) {
    return (float)read_temperature(sensor_buffer) / (float)256.0;
}

static void float_to_string(float number, char *buffer, int decimal_places)
{
    // Calculate the integer part
    int integer_part = (int)number;

    // Calculate the fractional part
    int multiplier = 1;
    for (int i = 0; i < decimal_places; i++)
    {
        multiplier *= 10;
    }

    int fractional_part = (int)((number - integer_part) * multiplier);

    // Handle negative numbers
    if (fractional_part < 0)
    {
        fractional_part = -fractional_part;
    }

    // Format the string with integer and fractional parts
    snprintf(buffer, 32, "%d.%0*d", integer_part, decimal_places, fractional_part);
}
