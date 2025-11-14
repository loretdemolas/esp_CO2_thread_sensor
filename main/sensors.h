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

void start_sensor_task(void);


#ifdef __cplusplus
}
#endif
