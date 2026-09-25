/** @file b4_imu.c
 *
 * @brief LSM303DLHC accelerometer access through the project I2C device.
 *
 * The ROBO-PICO I2C1 driver maps Grove Port 2 to GP2/GP3 and uses a fixed
 * 400 kHz bus. The adapter initializes unit 1 when it is not already
 * registered. Samples use
 * the datasheet's 100 Hz, plus or minus 2 g, high-resolution configuration.
 * The six-byte burst returns calibrated acceleration in milli-g. The caller
 * must own the I2C unit while this adapter is active.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tk/device.h>

#include "b4_imu.h"

#define B4_IMU_REGISTER_CTRL_REG1_A       0x20U
#define B4_IMU_REGISTER_CTRL_REG4_A       0x23U
#define B4_IMU_REGISTER_OUT_X_L_A         0x28U
#define B4_IMU_AUTO_INCREMENT             0x80U
#define B4_IMU_CTRL_REG1_100_HZ_XYZ       0x57U
#define B4_IMU_CTRL_REG4_BDU_HR_PLUS_2G   0x88U
#define B4_IMU_RAW_AXIS_SIGN_BIT          0x0800U
#define B4_IMU_RAW_AXIS_MODULUS            4096
#define B4_IMU_RAW_AXIS_SHIFT                 4U
#define B4_IMU_BURST_SIZE                     6U

typedef struct
{
    b4_cfg_config_t config;
    ID device_id;
    b4_types_health_t health;
    b4_types_fault_t fault;
    uint8_t consecutive_read_failures;
    bool b_initialized;
} b4_imu_state_t;

static b4_imu_state_t g_b4_imu_state = {0};

/** @brief Open the I2C device and initialize unit 1 when needed.
 *
 * @return The device identifier or a negative kernel error.
 */
static ID
b4_imu_open_device (void);

/** @brief Close the currently stored device handle.
 *
 * @return The close result from the kernel.
 */
static ER
b4_imu_close_device (void);

/** @brief Execute one I2C read transaction through the project device API.
 *
 * The device layer rejects a zero receive size, so receive_size is always at
 * least one byte. Register writes use b4_imu_write_register instead.
 *
 * @param p_send_data The transaction's register and write bytes.
 * @param send_size The number of bytes to send.
 * @param p_receive_data The destination for received bytes.
 * @param receive_size The number of bytes to receive.
 * @return The kernel or device-driver result.
 */
static ER
b4_imu_transfer (UB * p_send_data,
                 SZ send_size,
                 UB * p_receive_data,
                 SZ receive_size);

/** @brief Read one accelerometer register.
 *
 * @param register_address The seven-bit register address.
 * @param p_value The destination for the register value.
 * @return The transaction result.
 */
static ER
b4_imu_read_register (UB register_address, UB * p_value);

/** @brief Write one accelerometer register.
 *
 * @param register_address The seven-bit register address.
 * @param value The byte written to the register.
 * @return The transaction result.
 */
static ER
b4_imu_write_register (UB register_address, UB value);

/** @brief Configure the accelerometer and verify its control registers.
 *
 * @return The configuration transaction result.
 */
static ER
b4_imu_configure_sensor (void);

/** @brief Convert one left-justified, signed 12-bit sample to milli-g.
 *
 * @param low_byte The lower output-register byte.
 * @param high_byte The upper output-register byte.
 * @return The signed acceleration in milli-g.
 */
static int32_t
b4_imu_decode_axis (UB low_byte, UB high_byte);

/** @brief Convert the six output bytes to one calibrated sample.
 *
 * Each axis is corrected with the stored bias for that axis, where the bias
 * is the stationary residual, measured minus expected. See b4_cfg.h.
 *
 * @param p_data The six-byte X, Y, and Z burst.
 * @param p_timestamp_ms The monotonic time in milliseconds.
 * @param p_sample The destination for the calibrated sample.
 */
static void
b4_imu_decode_sample (const UB * p_data,
                      uint32_t timestamp_ms,
                      b4_types_sample_t * p_sample);

/** @brief Update sensor health after one failed read transaction.
 *
 * @param error The kernel or device-driver error.
 * @return Timeout, communication, or repeated-failure status.
 */
static b4_types_result_t
b4_imu_record_read_error (ER error);

/** @brief Initialize the accelerometer through the project I2C device.
 *
 * Calling this function while the adapter is already initialized returns
 * success without opening, reconfiguring, or resetting the device state, so
 * the configuration supplied by the first successful call stays in effect.
 * Calibrated bias values reach the adapter only after b4_imu_close followed
 * by another initialization with the updated configuration.
 *
 * @param p_config The validated hardware and calibration configuration.
 * @return The initialization result.
 */
b4_types_result_t
b4_imu_init (const b4_cfg_config_t * p_config)
{
    b4_types_result_t result = B4_RESULT_INVALID_ARGUMENT;

    if (NULL != p_config)
    {
        if (true == g_b4_imu_state.b_initialized)
        {
            /* Repeated initialization keeps the open device and its
             * configuration untouched. Close the adapter first to apply
             * newly calibrated bias values. */
            result = B4_RESULT_OK;
        }
        else
        {
            result = b4_cfg_validate(p_config);
            if (B4_RESULT_OK == result)
            {
                ER sensor_error = E_OK;

                g_b4_imu_state.config = *p_config;
                g_b4_imu_state.device_id = b4_imu_open_device();

                if (E_OK > g_b4_imu_state.device_id)
                {
                    result = B4_RESULT_SENSOR_COMM_FAILURE;
                    g_b4_imu_state.health = B4_HEALTH_FAULT;
                    g_b4_imu_state.fault = B4_FAULT_SENSOR_COMM;
                }
                else
                {
                    sensor_error = b4_imu_configure_sensor();
                    if (E_OK == sensor_error)
                    {
                        g_b4_imu_state.b_initialized = true;
                        g_b4_imu_state.health = B4_HEALTH_READY;
                        g_b4_imu_state.fault = B4_FAULT_NONE;
                        g_b4_imu_state.consecutive_read_failures = 0U;
                        result = B4_RESULT_OK;
                    }
                    else
                    {
                        (void)b4_imu_close_device();
                        g_b4_imu_state.device_id = 0;
                        g_b4_imu_state.health = B4_HEALTH_FAULT;
                        g_b4_imu_state.fault = B4_FAULT_SENSOR_COMM;
                        result = B4_RESULT_SENSOR_COMM_FAILURE;
                    }
                }
            }
            else
            {
                g_b4_imu_state.health = B4_HEALTH_FAULT;
                g_b4_imu_state.fault = B4_FAULT_HARDWARE_CONFIG;
            }
        }
    }

    return (result);
}

/** @brief Read and calibrate one accelerometer sample.
 *
 * @param p_sample The destination for the calibrated sample.
 * @return The sample result.
 */
b4_types_result_t
b4_imu_read (b4_types_sample_t * p_sample)
{
    b4_types_result_t result = B4_RESULT_INVALID_ARGUMENT;

    if (NULL != p_sample)
    {
        p_sample->b_valid = false;

        if (true == g_b4_imu_state.b_initialized)
        {
            UB register_address =
                (UB)(B4_IMU_REGISTER_OUT_X_L_A | B4_IMU_AUTO_INCREMENT);
            UB data[B4_IMU_BURST_SIZE] = {0U};
            SYSTIM system_time = {0};
            ER sensor_error = b4_imu_transfer(
                &register_address,
                (SZ)sizeof(register_address),
                data,
                (SZ)sizeof(data));

            if (E_OK == sensor_error)
            {
                sensor_error = tk_get_otm(&system_time);
            }

            if (E_OK == sensor_error)
            {
                b4_imu_decode_sample(
                    data, (uint32_t)system_time.lo, p_sample);
                g_b4_imu_state.consecutive_read_failures = 0U;
                g_b4_imu_state.health = B4_HEALTH_READY;
                g_b4_imu_state.fault = B4_FAULT_NONE;
                result = B4_RESULT_OK;
            }
            else
            {
                result = b4_imu_record_read_error(sensor_error);
            }
        }
        else
        {
            result = B4_RESULT_NOT_INITIALIZED;
        }
    }

    return (result);
}

/** @brief Copy sensor health, fault, and consecutive read failure count.
 *
 * @param p_health The destination for the current sensor health.
 * @param p_fault The destination for the current sensor fault.
 * @param p_failure_count The destination for the consecutive failure count.
 * @return The status result.
 */
b4_types_result_t
b4_imu_get_health (b4_types_health_t * p_health,
                   b4_types_fault_t * p_fault,
                   uint8_t * p_failure_count)
{
    b4_types_result_t result = B4_RESULT_OK;

    if ((NULL == p_health) || (NULL == p_fault) ||
        (NULL == p_failure_count))
    {
        result = B4_RESULT_INVALID_ARGUMENT;
    }
    else
    {
        *p_health = g_b4_imu_state.health;
        *p_fault = g_b4_imu_state.fault;
        *p_failure_count = g_b4_imu_state.consecutive_read_failures;

        if (false == g_b4_imu_state.b_initialized)
        {
            result = B4_RESULT_NOT_INITIALIZED;
        }
    }

    return (result);
}

/** @brief Power down the accelerometer and close its I2C device handle.
 *
 * @return The close result.
 */
b4_types_result_t
b4_imu_close (void)
{
    b4_types_result_t result = B4_RESULT_OK;

    if (false == g_b4_imu_state.b_initialized)
    {
        result = B4_RESULT_NOT_INITIALIZED;
    }
    else
    {
        ER sensor_error = b4_imu_write_register(
            B4_IMU_REGISTER_CTRL_REG1_A, 0U);
        ER close_error = b4_imu_close_device();

        if ((E_OK != sensor_error) || (E_OK != close_error))
        {
            result = B4_RESULT_SENSOR_COMM_FAILURE;
        }

        g_b4_imu_state.device_id = 0;
        g_b4_imu_state.b_initialized = false;
        g_b4_imu_state.health = B4_HEALTH_NOT_INITIALIZED;
        g_b4_imu_state.fault = B4_FAULT_NONE;
        g_b4_imu_state.consecutive_read_failures = 0U;
    }

    return (result);
}

/** @brief Open the I2C device and initialize unit 1 when needed.
 *
 * @return The device identifier or a negative kernel error.
 */
static ID
b4_imu_open_device (void)
{
    ID device_id = E_OK;
    ER init_error = E_OK;
    UB device_name[5] = {0U};

    device_name[0] = (UB)'i';
    device_name[1] = (UB)'i';
    device_name[2] = (UB)'c';
    device_name[3] = (UB)((UW)'a' + g_b4_imu_state.config.i2c_unit);
    device_name[4] = 0U;
    device_id = tk_opn_dev(device_name, TD_UPDATE);

    if ((E_OK > device_id) &&
        (1U == g_b4_imu_state.config.i2c_unit))
    {
        init_error = dev_init_i2c(
            (UW)g_b4_imu_state.config.i2c_unit);
        if (E_OK == init_error)
        {
            device_id = tk_opn_dev(device_name, TD_UPDATE);
        }
    }

    return (device_id);
}

/** @brief Close the currently stored device handle.
 *
 * @return The close result from the kernel.
 */
static ER
b4_imu_close_device (void)
{
    ER close_error = E_OK;

    if (E_OK < g_b4_imu_state.device_id)
    {
        close_error = tk_cls_dev(g_b4_imu_state.device_id, 0U);
    }

    return (close_error);
}

/** @brief Execute one I2C transaction through the project device API.
 *
 * @param p_send_data The transaction's register and write bytes.
 * @param send_size The number of bytes to send.
 * @param p_receive_data The destination for received bytes.
 * @param receive_size The number of bytes to receive.
 * @return The kernel or device-driver result.
 */
static ER
b4_imu_transfer (UB * p_send_data,
                 SZ send_size,
                 UB * p_receive_data,
                 SZ receive_size)
{
    ER transfer_error = E_OK;
    SZ reported_size = 0;
    T_I2C_EXEC transaction = {0};

    transaction.sadr =
        (UW)g_b4_imu_state.config.accel_i2c_address;
    transaction.snd_size = send_size;
    transaction.snd_data = p_send_data;
    transaction.rcv_size = receive_size;
    transaction.rcv_data = p_receive_data;

    transfer_error = tk_swri_dev(
        g_b4_imu_state.device_id,
        TDN_I2C_EXEC,
        &transaction,
        (SZ)sizeof(transaction),
        &reported_size);

    /* The I2C device layer reports the request-block size rather than the
     * number of payload bytes, so the transfer result is the completion
     * check. The size is still checked to confirm the request block was
     * consumed. sizeof(T_I2C_EXEC) is 20 bytes and fits the SZ range. */
    if ((E_OK == transfer_error) &&
        (reported_size != (SZ)sizeof(transaction)))
    {
        transfer_error = E_IO;
    }

    return (transfer_error);
}

/** @brief Read one accelerometer register.
 *
 * @param register_address The seven-bit register address.
 * @param p_value The destination for the register value.
 * @return The transaction result.
 */
static ER
b4_imu_read_register (UB register_address, UB * p_value)
{
    ER read_error = E_OK;

    read_error = b4_imu_transfer(
        &register_address,
        (SZ)sizeof(register_address),
        p_value,
        (SZ)sizeof(*p_value));

    return (read_error);
}

/** @brief Write one accelerometer register.
 *
 * The T-Kernel I2C device layer rejects a zero receive size for a
 * TDN_I2C_EXEC transaction, so a register write uses the device layer's
 * register-write path. That path sends the register address and value as a
 * plain write and needs no receive byte.
 *
 * @param register_address The seven-bit register address.
 * @param value The byte written to the register.
 * @return The register-write result.
 */
static ER
b4_imu_write_register (UB register_address, UB value)
{
    return (i2c_write_reg(g_b4_imu_state.device_id,
                          g_b4_imu_state.config.accel_i2c_address,
                          register_address,
                          value));
}

/** @brief Configure the accelerometer and verify its control registers.
 *
 * @return The configuration transaction result.
 */
static ER
b4_imu_configure_sensor (void)
{
    ER config_error = b4_imu_write_register(
        B4_IMU_REGISTER_CTRL_REG1_A,
        B4_IMU_CTRL_REG1_100_HZ_XYZ);
    UB control_reg1 = 0U;
    UB control_reg4 = 0U;

    if (E_OK == config_error)
    {
        config_error = b4_imu_write_register(
            B4_IMU_REGISTER_CTRL_REG4_A,
            B4_IMU_CTRL_REG4_BDU_HR_PLUS_2G);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_read_register(
            B4_IMU_REGISTER_CTRL_REG1_A, &control_reg1);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_read_register(
            B4_IMU_REGISTER_CTRL_REG4_A, &control_reg4);
    }

    if ((E_OK == config_error) &&
        ((B4_IMU_CTRL_REG1_100_HZ_XYZ != control_reg1) ||
         (B4_IMU_CTRL_REG4_BDU_HR_PLUS_2G != control_reg4)))
    {
        config_error = E_IO;
    }

    return (config_error);
}

/** @brief Convert one left-justified, signed 12-bit sample to milli-g.
 *
 * @param low_byte The lower output-register byte.
 * @param high_byte The upper output-register byte.
 * @return The signed acceleration in milli-g.
 */
static int32_t
b4_imu_decode_axis (UB low_byte, UB high_byte)
{
    uint16_t axis_word = (uint16_t)(
        ((uint16_t)high_byte << 8U) | (uint16_t)low_byte);
    uint16_t raw_value =
        (uint16_t)(axis_word >> B4_IMU_RAW_AXIS_SHIFT);
    int32_t acceleration_mg = (int32_t)raw_value;

    if (0U != (raw_value & B4_IMU_RAW_AXIS_SIGN_BIT))
    {
        acceleration_mg -= B4_IMU_RAW_AXIS_MODULUS;
    }

    return (acceleration_mg);
}

/** @brief Convert the six output bytes to one calibrated sample.
 *
 * Each axis is corrected with the stored bias for that axis, where the bias
 * is the stationary residual, measured minus expected. See b4_cfg.h.
 *
 * @param p_data The six-byte X, Y, and Z burst.
 * @param p_timestamp_ms The monotonic time in milliseconds.
 * @param p_sample The destination for the calibrated sample.
 */
static void
b4_imu_decode_sample (const UB * p_data,
                      uint32_t p_timestamp_ms,
                      b4_types_sample_t * p_sample)
{
    p_sample->timestamp_ms = p_timestamp_ms;
    p_sample->accel_x_mg = b4_imu_decode_axis(p_data[0], p_data[1]) -
                           g_b4_imu_state.config.bias_x_mg;
    p_sample->accel_y_mg = b4_imu_decode_axis(p_data[2], p_data[3]) -
                           g_b4_imu_state.config.bias_y_mg;
    p_sample->accel_z_mg = b4_imu_decode_axis(p_data[4], p_data[5]) -
                           g_b4_imu_state.config.bias_z_mg;
    p_sample->b_valid = true;
}

/** @brief Update sensor health after one failed read transaction.
 *
 * @param error The kernel or device-driver error.
 * @return Timeout, communication, or repeated-failure status.
 */
static b4_types_result_t
b4_imu_record_read_error (ER error)
{
    b4_types_result_t result = B4_RESULT_SENSOR_COMM_FAILURE;

    if (g_b4_imu_state.consecutive_read_failures <
        g_b4_imu_state.config.max_consecutive_read_failures)
    {
        g_b4_imu_state.consecutive_read_failures++;
    }

    if (E_TMOUT == error)
    {
        g_b4_imu_state.fault = B4_FAULT_READ_TIMEOUT;
        result = B4_RESULT_TIMEOUT;
    }
    else
    {
        g_b4_imu_state.fault = B4_FAULT_SENSOR_COMM;
        result = B4_RESULT_SENSOR_COMM_FAILURE;
    }

    g_b4_imu_state.health = B4_HEALTH_DEGRADED;
    if (g_b4_imu_state.consecutive_read_failures >=
        g_b4_imu_state.config.max_consecutive_read_failures)
    {
        g_b4_imu_state.health = B4_HEALTH_FAULT;
        g_b4_imu_state.fault = B4_FAULT_REPEATED_READ;
        result = B4_RESULT_REPEATED_READ_FAILURE;
    }

    return (result);
}
