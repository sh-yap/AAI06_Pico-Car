/** @file b4_imu.c
 *
 * @brief LSM303DLHC accelerometer and magnetometer access through the I2C
 * device.
 *
 * The ROBO-PICO I2C1 driver maps Grove port 2 to SDA=GP2 and SCL=GP3 and
 * uses a fixed 400 kHz bus. The adapter initializes unit 1 when it is not
 * already registered. Accelerometer samples use the datasheet's 100 Hz, plus
 * or minus 2 g, high-resolution configuration; the six-byte burst returns
 * calibrated acceleration in milli-g. Magnetometer samples use continuous
 * conversion at the configured rate and gain and return raw sensor counts.
 * The caller must own the I2C unit while this adapter is active.
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
#define B4_IMU_REGISTER_CRA_REG_M         0x00U
#define B4_IMU_REGISTER_CRB_REG_M         0x01U
#define B4_IMU_REGISTER_MR_REG_M          0x02U
#define B4_IMU_REGISTER_OUT_X_H_M         0x03U
#define B4_IMU_REGISTER_SR_REG_M          0x09U
#define B4_IMU_CRA_MAG_RATE_30HZ          0x14U
#define B4_IMU_CRA_MAG_RATE_75HZ          0x18U
#define B4_IMU_CRB_GAIN_1_3_GAUSS         0x20U
#define B4_IMU_MR_MAG_CONTINUOUS          0x00U
#define B4_IMU_MR_MAG_SLEEP               0x03U
#define B4_IMU_SR_MAG_DATA_READY          0x01U
#define B4_IMU_MAG_BURST_SIZE                 6U
#define B4_IMU_MAG_MAX_COUNTS              4096

typedef struct
{
    b4_cfg_config_t config;
    ID device_id;
    b4_types_health_t health;
    b4_types_fault_t fault;
    b4_types_health_t mag_health;
    b4_types_fault_t mag_fault;
    uint8_t consecutive_read_failures;
    uint8_t mag_consecutive_read_failures;
    bool b_initialized;
    bool b_mag_available;
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
 * @param slave_address The seven-bit address of the selected sensor.
 * @param p_send_data The transaction's register and write bytes.
 * @param send_size The number of bytes to send.
 * @param p_receive_data The destination for received bytes.
 * @param receive_size The number of bytes to receive.
 * @return The kernel or device-driver result.
 */
static ER
b4_imu_transfer (UW slave_address,
                 UB * p_send_data,
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

/** @brief Configure the magnetometer and verify its control registers.
 *
 * @return The configuration transaction result.
 */
static ER
b4_imu_configure_magnetometer (void);

/** @brief Convert a configured magnetometer rate to its register value.
 *
 * @param rate_hz The configured rate in hertz.
 * @return The CRA_REG_M data output rate bits.
 */
static UB
b4_imu_mag_rate_register (uint8_t rate_hz);

/** @brief Read one magnetometer register.
 *
 * @param register_address The seven-bit register address.
 * @param p_value The destination for the register value.
 * @return The transaction result.
 */
static ER
b4_imu_read_mag_register (UB register_address, UB * p_value);

/** @brief Write one magnetometer register.
 *
 * @param register_address The seven-bit register address.
 * @param value The byte written to the register.
 * @return The register-write result.
 */
static ER
b4_imu_write_mag_register (UB register_address, UB value);

/** @brief Convert two big-endian magnetometer bytes to signed counts.
 *
 * @param high_byte The upper output-register byte.
 * @param low_byte The lower output-register byte.
 * @return The signed field value in sensor counts.
 */
static int16_t
b4_imu_decode_mag_axis (UB high_byte, UB low_byte);

/** @brief Check that every magnetometer axis is within the output span.
 *
 * @param p_mag_sample The decoded sample to check.
 * @return True when each axis is within the 12-bit output span.
 */
static bool
b4_imu_mag_is_in_range (const b4_types_mag_sample_t * p_mag_sample);

/** @brief Update magnetometer health after one failed read transaction.
 *
 * @param error The kernel or device-driver error.
 * @return Timeout, communication, or repeated-failure status.
 */
static b4_types_result_t
b4_imu_record_mag_error (ER error);

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
                        ER mag_error = b4_imu_configure_magnetometer();

                        if (E_OK == mag_error)
                        {
                            g_b4_imu_state.b_mag_available = true;
                            g_b4_imu_state.mag_health = B4_HEALTH_READY;
                            g_b4_imu_state.mag_fault = B4_FAULT_NONE;
                            g_b4_imu_state.mag_consecutive_read_failures = 0U;
                        }
                        else
                        {
                            /* The accelerometer still works, so terrain
                             * detection and height stay available. Only
                             * heading is reported as unavailable. */
                            g_b4_imu_state.b_mag_available = false;
                            g_b4_imu_state.mag_health = B4_HEALTH_FAULT;
                            g_b4_imu_state.mag_fault = B4_FAULT_MAGNETOMETER;
                        }

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
                (UW)g_b4_imu_state.config.accel_i2c_address,
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

/** @brief Read one magnetometer sample when the sensor has new data.
 *
 * The data-ready flag is checked first so a repeated or torn register set is
 * never reported as a fresh magnetic measurement.
 *
 * @param p_mag_sample The destination for the magnetometer sample.
 * @return The sample result.
 */
b4_types_result_t
b4_imu_read_magnetometer (b4_types_mag_sample_t * p_mag_sample)
{
    b4_types_result_t result = B4_RESULT_INVALID_ARGUMENT;

    if (NULL != p_mag_sample)
    {
        UB status_register = 0U;

        p_mag_sample->b_valid = false;

        if (false == g_b4_imu_state.b_initialized)
        {
            result = B4_RESULT_NOT_INITIALIZED;
        }
        else if (false == g_b4_imu_state.b_mag_available)
        {
            result = B4_RESULT_SENSOR_COMM_FAILURE;
        }
        else
        {
            ER sensor_error = b4_imu_read_mag_register(
                B4_IMU_REGISTER_SR_REG_M, &status_register);

            if (E_OK != sensor_error)
            {
                result = b4_imu_record_mag_error(sensor_error);
            }
            else if (0U == (status_register & B4_IMU_SR_MAG_DATA_READY))
            {
                result = B4_RESULT_STALE_SAMPLE;
            }
            else
            {
                UB register_address = (UB)(
                    B4_IMU_REGISTER_OUT_X_H_M | B4_IMU_AUTO_INCREMENT);
                UB data[B4_IMU_MAG_BURST_SIZE] = {0U};
                SYSTIM system_time = {0};

                sensor_error = b4_imu_transfer(
                    (UW)g_b4_imu_state.config.mag_i2c_address,
                    &register_address,
                    (SZ)sizeof(register_address),
                    data,
                    (SZ)sizeof(data));

                if (E_OK == sensor_error)
                {
                    sensor_error = tk_get_otm(&system_time);
                }

                if (E_OK != sensor_error)
                {
                    result = b4_imu_record_mag_error(sensor_error);
                }
                else
                {
                    /* The magnetometer register map orders the axes X, Z,
                     * then Y, unlike the accelerometer map. */
                    p_mag_sample->timestamp_ms = (uint32_t)system_time.lo;
                    p_mag_sample->mag_x = b4_imu_decode_mag_axis(
                        data[0], data[1]);
                    p_mag_sample->mag_z = b4_imu_decode_mag_axis(
                        data[2], data[3]);
                    p_mag_sample->mag_y = b4_imu_decode_mag_axis(
                        data[4], data[5]);

                    if (true == b4_imu_mag_is_in_range(p_mag_sample))
                    {
                        p_mag_sample->b_valid = true;
                        g_b4_imu_state.mag_consecutive_read_failures = 0U;
                        g_b4_imu_state.mag_health = B4_HEALTH_READY;
                        g_b4_imu_state.mag_fault = B4_FAULT_NONE;
                        result = B4_RESULT_OK;
                    }
                    else
                    {
                        result = B4_RESULT_INVALID_SAMPLE;
                    }
                }
            }
        }
    }

    return (result);
}

/** @brief Copy magnetometer health, fault, and consecutive failure count.
 *
 * @param p_health The destination for the current magnetometer health.
 * @param p_fault The destination for the current magnetometer fault.
 * @param p_failure_count The destination for the consecutive failure count.
 * @return The status result.
 */
b4_types_result_t
b4_imu_get_mag_health (b4_types_health_t * p_health,
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
        *p_health = g_b4_imu_state.mag_health;
        *p_fault = g_b4_imu_state.mag_fault;
        *p_failure_count = g_b4_imu_state.mag_consecutive_read_failures;

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
        ER mag_error = E_OK;
        ER close_error = E_OK;

        if (true == g_b4_imu_state.b_mag_available)
        {
            mag_error = b4_imu_write_mag_register(
                B4_IMU_REGISTER_MR_REG_M, B4_IMU_MR_MAG_SLEEP);
        }

        close_error = b4_imu_close_device();

        if ((E_OK != sensor_error) || (E_OK != mag_error) ||
            (E_OK != close_error))
        {
            result = B4_RESULT_SENSOR_COMM_FAILURE;
        }

        g_b4_imu_state.device_id = 0;
        g_b4_imu_state.b_initialized = false;
        g_b4_imu_state.health = B4_HEALTH_NOT_INITIALIZED;
        g_b4_imu_state.fault = B4_FAULT_NONE;
        g_b4_imu_state.consecutive_read_failures = 0U;
        g_b4_imu_state.b_mag_available = false;
        g_b4_imu_state.mag_health = B4_HEALTH_NOT_INITIALIZED;
        g_b4_imu_state.mag_fault = B4_FAULT_NONE;
        g_b4_imu_state.mag_consecutive_read_failures = 0U;
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
b4_imu_transfer (UW slave_address,
                 UB * p_send_data,
                 SZ send_size,
                 UB * p_receive_data,
                 SZ receive_size)
{
    ER transfer_error = E_OK;
    SZ reported_size = 0;
    T_I2C_EXEC transaction = {0};

    transaction.sadr = slave_address;
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
        (UW)g_b4_imu_state.config.accel_i2c_address,
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

/** @brief Configure the magnetometer and verify its control registers.
 *
 * @return The configuration transaction result.
 */
static ER
b4_imu_configure_magnetometer (void)
{
    UB rate_register = b4_imu_mag_rate_register(
        g_b4_imu_state.config.mag_data_rate_hz);
    ER config_error = b4_imu_write_mag_register(
        B4_IMU_REGISTER_CRA_REG_M, rate_register);
    UB control_a = 0U;
    UB control_b = 0U;
    UB mode = 0U;

    if (E_OK == config_error)
    {
        config_error = b4_imu_write_mag_register(
            B4_IMU_REGISTER_CRB_REG_M, B4_IMU_CRB_GAIN_1_3_GAUSS);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_write_mag_register(
            B4_IMU_REGISTER_MR_REG_M, B4_IMU_MR_MAG_CONTINUOUS);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_read_mag_register(
            B4_IMU_REGISTER_CRA_REG_M, &control_a);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_read_mag_register(
            B4_IMU_REGISTER_CRB_REG_M, &control_b);
    }

    if (E_OK == config_error)
    {
        config_error = b4_imu_read_mag_register(
            B4_IMU_REGISTER_MR_REG_M, &mode);
    }

    if ((E_OK == config_error) &&
        ((rate_register != control_a) ||
         (B4_IMU_CRB_GAIN_1_3_GAUSS != control_b) ||
         (B4_IMU_MR_MAG_CONTINUOUS != mode)))
    {
        config_error = E_IO;
    }

    return (config_error);
}

/** @brief Convert a configured magnetometer rate to its register value.
 *
 * @param rate_hz The configured rate in hertz.
 * @return The CRA_REG_M data output rate bits.
 */
static UB
b4_imu_mag_rate_register (uint8_t rate_hz)
{
    UB register_value = B4_IMU_CRA_MAG_RATE_30HZ;

    if (B4_CFG_MAG_RATE_75_HZ == rate_hz)
    {
        register_value = B4_IMU_CRA_MAG_RATE_75HZ;
    }

    return (register_value);
}

/** @brief Read one magnetometer register.
 *
 * @param register_address The seven-bit register address.
 * @param p_value The destination for the register value.
 * @return The transaction result.
 */
static ER
b4_imu_read_mag_register (UB register_address, UB * p_value)
{
    ER read_error = E_OK;

    read_error = b4_imu_transfer(
        (UW)g_b4_imu_state.config.mag_i2c_address,
        &register_address,
        (SZ)sizeof(register_address),
        p_value,
        (SZ)sizeof(*p_value));

    return (read_error);
}

/** @brief Write one magnetometer register.
 *
 * @param register_address The seven-bit register address.
 * @param value The byte written to the register.
 * @return The register-write result.
 */
static ER
b4_imu_write_mag_register (UB register_address, UB value)
{
    return (i2c_write_reg(g_b4_imu_state.device_id,
                          g_b4_imu_state.config.mag_i2c_address,
                          register_address,
                          value));
}

/** @brief Convert two big-endian magnetometer bytes to signed counts.
 *
 * @param high_byte The upper output-register byte.
 * @param low_byte The lower output-register byte.
 * @return The signed field value in sensor counts.
 */
static int16_t
b4_imu_decode_mag_axis (UB high_byte, UB low_byte)
{
    uint16_t axis_word = (uint16_t)(
        ((uint16_t)high_byte << 8U) | (uint16_t)low_byte);

    /* Both types are 16 bits wide, so this cast preserves the exact
     * two's complement bit pattern. */
    return ((int16_t)axis_word);
}

/** @brief Check that every magnetometer axis is within the output span.
 *
 * @param p_mag_sample The decoded sample to check.
 * @return True when each axis is within the 12-bit output span.
 */
static bool
b4_imu_mag_is_in_range (const b4_types_mag_sample_t * p_mag_sample)
{
    bool b_in_range = false;

    if ((B4_IMU_MAG_MAX_COUNTS > p_mag_sample->mag_x) &&
        (-B4_IMU_MAG_MAX_COUNTS < p_mag_sample->mag_x) &&
        (B4_IMU_MAG_MAX_COUNTS > p_mag_sample->mag_y) &&
        (-B4_IMU_MAG_MAX_COUNTS < p_mag_sample->mag_y) &&
        (B4_IMU_MAG_MAX_COUNTS > p_mag_sample->mag_z) &&
        (-B4_IMU_MAG_MAX_COUNTS < p_mag_sample->mag_z))
    {
        b_in_range = true;
    }

    return (b_in_range);
}

/** @brief Update magnetometer health after one failed read transaction.
 *
 * @param error The kernel or device-driver error.
 * @return Timeout, communication, or repeated-failure status.
 */
static b4_types_result_t
b4_imu_record_mag_error (ER error)
{
    b4_types_result_t result = B4_RESULT_SENSOR_COMM_FAILURE;

    if (g_b4_imu_state.mag_consecutive_read_failures <
        g_b4_imu_state.config.max_consecutive_read_failures)
    {
        g_b4_imu_state.mag_consecutive_read_failures++;
    }

    if (E_TMOUT == error)
    {
        g_b4_imu_state.mag_fault = B4_FAULT_READ_TIMEOUT;
        result = B4_RESULT_TIMEOUT;
    }
    else
    {
        g_b4_imu_state.mag_fault = B4_FAULT_SENSOR_COMM;
        result = B4_RESULT_SENSOR_COMM_FAILURE;
    }

    g_b4_imu_state.mag_health = B4_HEALTH_DEGRADED;
    if (g_b4_imu_state.mag_consecutive_read_failures >=
        g_b4_imu_state.config.max_consecutive_read_failures)
    {
        g_b4_imu_state.mag_health = B4_HEALTH_FAULT;
        g_b4_imu_state.mag_fault = B4_FAULT_REPEATED_READ;
        result = B4_RESULT_REPEATED_READ_FAILURE;
    }

    return (result);
}
