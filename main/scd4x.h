/*
 * scd4x.h - SCD4x driver (ESP-IDF I2C v2)
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SCD4X_I2C_ADDRESS 0x62
#define SCD4X_I2C_FREQ_HZ 100000

/** Handle for SCD4x using I2C v2 */
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} scd4x_handle_t;

/**
 * @brief Initialize a SCD4x sensor handle on an existing I2C bus.
 *
 * This function attaches the SCD4x device to a pre-created I2C master bus.
 *
 * @param sensor_out Pointer to an uninitialized scd4x_handle_t structure. Must not be NULL.
 * @param bus_handle Handle to an existing I2C master bus created with i2c_new_master_bus().
 * 
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if sensor_out or bus_handle is NULL
 * - Other esp_err_t codes if I2C device creation fails
 */
esp_err_t scd4x_init(scd4x_handle_t *sensor_out, i2c_master_bus_handle_t bus_handle);

/**
 * @brief Create a new I2C master bus and initialize a SCD4x sensor on it.
 *
 * This convenience function sets up a new I2C bus using I2C_NUM_0, configures SDA/SCL pins, 
 * and initializes the sensor.
 *
 * @param bus_out Pointer to an uninitialized i2c_master_bus_handle_t structure. Must not be NULL.
 * @param sda_gpio GPIO number for I2C SDA line.
 * @param scl_gpio GPIO number for I2C SCL line.
 * 
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if sensor_out is NULL
 * - Other esp_err_t codes if I2C bus creation or sensor initialization fails
 */
esp_err_t scd4x_create_bus_and_init(i2c_master_bus_handle_t *bus_out, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free resources associated with a SCD4x sensor handle.
 *
 * This removes the I2C device handle. Optionally, the I2C bus can also be deleted.
 *
 * @param sensor Pointer to the initialized scd4x_handle_t structure. Must not be NULL.
 * @param free_bus If true, the I2C master bus associated with the sensor will also be deleted.
 * 
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if sensor is NULL
 */
esp_err_t scd4x_free(scd4x_handle_t *sensor, bool free_bus);

/*-------------------------------- Commands --------------------------------*/

/**
 * @brief Start periodic CO2, temperature, and humidity measurements.
 *
 * Measurements are taken automatically at approximately 5-second intervals.
 *
 * @param sensor Initialized SCD4x sensor handle.
 * 
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if sensor is NULL
 * - Other esp_err_t codes on I2C failure
 */
esp_err_t scd4x_start_periodic_measurement(scd4x_handle_t *sensor);

/**
 * @brief Stop periodic measurements.
 *
 * The sensor returns to idle mode. A new start command is required to resume measurement.
 *
 * @param sensor Initialized SCD4x sensor handle.
 *
 * @return ESP_OK or error code
 */
esp_err_t scd4x_stop_periodic_measurement(scd4x_handle_t *sensor);

/**
 * @brief Start low-power periodic measurements.
 *
 * Measurements are taken at approximately 30-second intervals to reduce power consumption.
 *
 * @param sensor Initialized SCD4x sensor handle.
 *
 * @return ESP_OK or error code
 */
esp_err_t scd4x_start_low_power_periodic_measurement(scd4x_handle_t *sensor);

/**
 * @brief Read raw measurement values from the sensor.
 *
 * Retrieves 16-bit raw words corresponding to CO2 concentration, temperature, and relative humidity.
 * Useful for advanced users or for applying custom conversions.
 *
 * @param sensor Initialized SCD4x sensor handle.
 * @param co2 Pointer to store raw CO2 value (ppm equivalent, raw ticks). Can be NULL.
 * @param temperature Pointer to store raw temperature value (ticks). Can be NULL.
 * @param humidity Pointer to store raw humidity value (ticks). Can be NULL.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd4x_read_measurement_ticks(scd4x_handle_t *sensor, uint16_t *co2, uint16_t *temperature, uint16_t *humidity);

/**
 * @brief Read processed measurement values from the sensor.
 *
 * Converts raw ticks into human-readable units:
 * - CO2 in ppm (uint16_t)
 * - Temperature in degrees Celsius (float)
 * - Relative humidity in %RH (float)
 *
 * @param sensor Initialized SCD4x sensor handle.
 * @param co2 Pointer to store CO2 concentration in ppm. Can be NULL.
 * @param temperature_c Pointer to store temperature in °C. Can be NULL.
 * @param humidity_rh Pointer to store relative humidity in %. Can be NULL.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd4x_read_measurement(scd4x_handle_t *sensor, uint16_t *co2, float *temperature_c, float *humidity_rh);

/**
 * @brief Check whether new measurement data is available.
 *
 * Can be called before reading measurements to avoid reading stale values.
 *
 * @param sensor Initialized SCD4x sensor handle.
 * @param data_ready Pointer to a bool that will be set to true if new data is ready.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd4x_get_data_ready_status(scd4x_handle_t *sensor, bool *data_ready);

/* Temperature offset */
esp_err_t scd4x_get_temperature_offset_ticks(scd4x_handle_t *sensor, uint16_t *t_offset);
esp_err_t scd4x_get_temperature_offset(scd4x_handle_t *sensor, float *t_offset);
esp_err_t scd4x_set_temperature_offset_ticks(scd4x_handle_t *sensor, uint16_t t_offset);
esp_err_t scd4x_set_temperature_offset(scd4x_handle_t *sensor, float t_offset);

/* Altitude and ambient pressure */
esp_err_t scd4x_get_sensor_altitude(scd4x_handle_t *sensor, uint16_t *altitude);
esp_err_t scd4x_set_sensor_altitude(scd4x_handle_t *sensor, uint16_t altitude);
esp_err_t scd4x_set_ambient_pressure(scd4x_handle_t *sensor, uint16_t pressure_hpa);

/* Forced recalibration (FRC) */
esp_err_t scd4x_perform_forced_recalibration(scd4x_handle_t *sensor, uint16_t target_co2_concentration, uint16_t *frc_correction);

/* Automatic self calibration (ASC) */
esp_err_t scd4x_get_automatic_self_calibration(scd4x_handle_t *sensor, bool *enabled);
esp_err_t scd4x_set_automatic_self_calibration(scd4x_handle_t *sensor, bool enabled);

/* Persist settings to EEPROM */
esp_err_t scd4x_persist_settings(scd4x_handle_t *sensor);

/* Serial number */
esp_err_t scd4x_get_serial_number(scd4x_handle_t *sensor, uint16_t *serial0, uint16_t *serial1, uint16_t *serial2);

/* Self-test, factory reset, reinit */
esp_err_t scd4x_perform_self_test(scd4x_handle_t *sensor, bool *malfunction);
esp_err_t scd4x_perform_factory_reset(scd4x_handle_t *sensor);
esp_err_t scd4x_reinit(scd4x_handle_t *sensor);

/* Single-shot measurements */
esp_err_t scd4x_measure_single_shot(scd4x_handle_t *sensor);
esp_err_t scd4x_measure_single_shot_rht_only(scd4x_handle_t *sensor);

/* Power down / wake up */
esp_err_t scd4x_power_down(scd4x_handle_t *sensor);
esp_err_t scd4x_wake_up(scd4x_handle_t *sensor);

#ifdef __cplusplus
}
#endif
