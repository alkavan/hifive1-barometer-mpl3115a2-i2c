#ifndef HIFIVE1_BAROMETER_MPL3115A2_I2C_SENSOR_H
#define HIFIVE1_BAROMETER_MPL3115A2_I2C_SENSOR_H

#include <kernel.h>
#include <device.h>

#define MPL3115A2_ADDRESS    (0x60)
#define MPL3115A2_8BIT_WRITE (0xC0)
#define MPL3115A2_8BIT_READ  (0xC1)

#define MPL3115A2_STATUS_DATA_RDY_BITMASK (1 << 3)

/**
 *  0x00 Sensor Status Register         (STATUS)(1)(2)          0x00 Yes R   0x01      Alias for DR_STATUS or F_STATUS.
 *  0x01 Pressure Data Out MSB          (OUT_P_MSB)(1)(2)       0x00 Yes R   0x02 0x01 Bits 12-19 of 20-bit real-time Pressure sample. Root pointer to Pressure and Temperature FIFO data.
 *  0x02 Pressure Data Out CSB          (OUT_P_CSB)(1)(2)       0x00 Yes R   0x03      Bits 4-11 of 20-bit real-time Pressure sample.
 *  0x03 Pressure Data Out LSB          (OUT_P_LSB)(1)(2)       0x00 Yes R   0x04      Bits 0-3 of 20-bit real-time Pressure sample.
 *  0x04 Temperature Data Out MSB       (OUT_T_MSB)(1)(2)       0x00 Yes R   0x05      Bits 4-11 of 12-bit real-time Temperature sample.
 *  0x05 Temperature Data Out LSB       (OUT_T_LSB)(1)(2)       0x00 Yes R   0x00      Bits 0-3 of 12-bit real-time Temperature sample.
 *  0x06/0x00 Sensor Status Register    (DR_STATUS)(1)(2)       0x00 Yes R   0x07      Data Ready status information.
 *  0x07 Pressure Data Out Delta MSB    (OUT_P_DELTA_MSB)(1)(2) 0x00 Yes R   0x08      Bits 12-19 of 20-bit Pressure change data.
 *  0x08 Pressure Data Out Delta CSB    (OUT_P_DELTA_CSB)(1)(2) 0x00 Yes R   0x09      Bits 4-11 of 20-bit Pressure change data.
 *  0x09 Pressure Data Out Delta LSB    (OUT_P_DELTA_LSB)(1)(2) 0x00 Yes R   0x0A      Bits 0-3 of 20-bit Pressure change data.
 *  0x0A Temperature Data Out Delta MSB (OUT_T_DELTA_MSB)(1)(2) 0x00 Yes R   0x0B      Bits 4-11 of 12-bit Temperature change data.
 *  0x0B Temperature Data Out Delta LSB (OUT_T_DELTA_LSB)(1)(2) 0x00 Yes R   0x06      Bits 0-3 of 12-bit Temperature change data.
 *  0x0C Device Identification Register (WHO_AM_I)              0xC4 No  R   0x0D      Fixed Device ID Number.
 *  0x0D FIFO Status Register           (F_STATUS)(1)(2)        0x00 Yes R   0x0E      FIFO Status: No FIFO event detected.
 *  0X0E/0x01 FIFO 8-bit Data Access    (F_DATA)(1)(2)          0x00 Yes R   0x0E      FIFO 8-bit data access.
 *  0x0F FIFO Setup Register            (F_SETUP)(1)(3)         0x00 No  R/W 0x10      FIFO setup.
 *  0x10 Time Delay Register            (TIME_DLY)(1)(2)        0x00 Yes R   0x11      Time since FIFO overflow.
 *  0x11 System Mode Register           (SYSMOD)(2)             0x00 Yes R   0x12      Current system mode.
 *  0x12 Interrupt Source Register      (INT_SOURCE)(1)         0x00 No  R   0x13      Interrupt status.
 *  0x13 PT Data Configuration Register (PT_DATA_CFG)(1)(3)     0x00 No  R/W 0x14      Data event flag configuration.
 *  0x14 BAR Input in MSB               (BAR_IN_MSB)(1)(3)      0xC5 No  R/W 0x15      Barometric input for Altitude calculation bits 8-15.
 *  0x15 BAR Input in LSB               (BAR_IN_LSB)(1)(3)      0xE7 No  R/W 0x16      Barometric input for Altitude.
 */

/**
 * @brief MPL3115A2 sensor registers.
 */
enum {
    MPL3115A2_REGISTER_STATUS = (0x00),

    // pressure
    MPL3115A2_REGISTER_PRESSURE_MSB = (0x01),
    MPL3115A2_REGISTER_PRESSURE_CSB = (0x02),
    MPL3115A2_REGISTER_PRESSURE_LSB = (0x03),

    // temperature
    MPL3115A2_REGISTER_TEMP_MSB = (0x04),
    MPL3115A2_REGISTER_TEMP_LSB = (0x05),

    // status
    MPL3115A2_REGISTER_DR_STATUS = (0x06),

    // pressure delta
    MPL3115A2_OUT_P_DELTA_MSB = (0x07),
    MPL3115A2_OUT_P_DELTA_CSB = (0x08),
    MPL3115A2_OUT_P_DELTA_LSB = (0x09),

    // temperature delta
    MPL3115A2_OUT_T_DELTA_MSB = (0x0A),
    MPL3115A2_OUT_T_DELTA_LSB = (0x0B),

    MPL3115A2_WHOAMI = (0x0C),

    MPL3115A2_BAR_IN_MSB = (0x14),
    MPL3115A2_BAR_IN_LSB = (0x15),

    MPL3115A2_OFF_H = (0x2D),
};

/**
 * @brief MPL3115A2 sensor status register bits.
 */
enum {
    MPL3115A2_REGISTER_STATUS_TDR  = 0x02,
    MPL3115A2_REGISTER_STATUS_PDR  = 0x04,
    MPL3115A2_REGISTER_STATUS_PTDR = 0x08,
};

/**
 * @brief MPL3115A2 sensor PT DATA register bits.
 */
enum {
    MPL3115A2_PT_DATA_CFG       = 0x13,
    MPL3115A2_PT_DATA_CFG_TDEFE = 0x01,
    MPL3115A2_PT_DATA_CFG_PDEFE = 0x02,
    MPL3115A2_PT_DATA_CFG_DREM  = 0x04,
};

/**
 * @brief MPL3115A2 sensor control registers.
 */
enum {

    MPL3115A2_CTRL_REG1 = (0x26),
    MPL3115A2_CTRL_REG2 = (0x27),
    MPL3115A2_CTRL_REG3 = (0x28),
    MPL3115A2_CTRL_REG4 = (0x29),
    MPL3115A2_CTRL_REG5 = (0x2A),
};

/**
 * @brief MPL3115A2 sensor control register bits.
 */
enum {
    MPL3115A2_CTRL_REG1_BAR  = 0x00,
    MPL3115A2_CTRL_REG1_SBYB = 0x01,
    MPL3115A2_CTRL_REG1_OST  = 0x02,
    MPL3115A2_CTRL_REG1_RST  = 0x04,
    MPL3115A2_CTRL_REG1_RAW  = 0x40,
    MPL3115A2_CTRL_REG1_ALT  = 0x80,
};

/**
 * @brief MPL3115A2 sensor oversample values.
 */
enum {
    MPL3115A2_CTRL_REG1_OS1   = 0x00,
    MPL3115A2_CTRL_REG1_OS2   = 0x08,
    MPL3115A2_CTRL_REG1_OS4   = 0x10,
    MPL3115A2_CTRL_REG1_OS8   = 0x18,
    MPL3115A2_CTRL_REG1_OS16  = 0x20,
    MPL3115A2_CTRL_REG1_OS32  = 0x28,
    MPL3115A2_CTRL_REG1_OS64  = 0x30,
    MPL3115A2_CTRL_REG1_OS128 = 0x38,
};

/**
 * @brief MPL3115A2 sensor measurement modes
 */
typedef enum {
    MPL3115A2_BAROMETER = 0,
    MPL3115A2_ALTIMETER = 1,
} mpl3115a2_mode_t;

/**
 * @brief MPL3115A2 sensor measurement types
 */
typedef enum {
    MPL3115A2_PRESSURE    = 0,
    MPL3115A2_ALTITUDE    = 1,
    MPL3115A2_TEMPERATURE = 2,
} mpl3115a2_meas_t;

/**
 * @brief Set sensor configuration flag.
 *
 * @return
 */
uint32_t sensor_get_config();

/**
 * @brief Configure and activate I2C sensor device.
 *
 * @param i2c_dev
 * @param i2c_cfg
 * @return true if device was configured and ready to use
 */
bool activate_mpl3115a2_device(const struct device * i2c_dev, uint32_t i2c_cfg);

/**
 * @brief Do sensor activation procedure.
 *
 * @param i2c_dev
 * @return
 */
bool setup_mpl3115a2_device(const struct device * i2c_dev);

/**
 * @brief Do sensor read cycle procedure
 *
 * @param i2c_dev
 * @param sensor_error
 * @param sensor_buffer
 * @param sleep_ms
 * @return
 */
bool do_mpl3115a2_cycle(
        const struct device * i2c_dev,
        int32_t * sensor_error,
        uint8_t * sensor_buffer,
        uint32_t sleep_ms
        );

/**
 * @brief Read value from the sensor register.
 *
 * @param i2c_dev
 * @param reg_addr
 * @param value
 * @return
 */
int sensor_read_8bit(const struct device * i2c_dev, uint8_t reg_addr, uint8_t * value);

/**
 * @brief Write data to sensor register.
 *
 * @param i2c_dev
 * @param reg_addr
 * @param value
 * @return
 */
int sensor_write_8bit(const struct device * i2c_dev, uint8_t reg_addr, uint8_t value);

/**
 * @brief Reset the sensor error array.
 *
 * @param sensor_error
 * @param size
 */
void sensor_reset_errors(int32_t * sensor_error, size_t size);

/**
 * @brief Sum all opcodes in the error array.
 *
 * @param sensor_error
 * @param size
 * @return
 */
uint32_t sensor_sum_errors(const int32_t * sensor_error, size_t size);

#endif //HIFIVE1_BAROMETER_MPL3115A2_I2C_SENSOR_H
