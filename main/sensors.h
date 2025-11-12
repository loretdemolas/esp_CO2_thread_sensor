#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the SCD4x sensor over I2C.
 * Must be called before reading values.
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_init(void);

/**
 * Read CO2 (ppm), temperature (°C), and humidity (%RH)
 * @param co2_ppm Pointer to float for CO2 ppm
 * @param temperature Pointer to float for temperature in Celsius
 * @param humidity Pointer to float for relative humidity
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_read(float *co2_ppm, float *temperature, float *humidity);

/**
 * Deinitialize the sensor and free I2C resources
 */
void sensors_deinit(void);

void start_sensor_task(void);


#ifdef __cplusplus
}
#endif
