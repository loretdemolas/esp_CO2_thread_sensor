#include "sensors.h"
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "scd4x.h"

static const char *TAG = "sensors";

/* Seeed Studio XIAO ESP32C6 I2C Pins */
#define I2C_SDA_IO       22
#define I2C_SCL_IO       23

/* Handles for new I2C driver */
static i2c_master_bus_handle_t i2c_bus = NULL;
static scd4x_handle_t scd4x_handle;

/* Initialize sensor */
esp_err_t sensors_init(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Creating I2C bus and initializing SCD4x...");
    err = scd4x_create_bus_and_init(&i2c_bus, I2C_SDA_IO, I2C_SCL_IO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create bus and init SCD4x: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Initializing SCD4x handle...");
    err = scd4x_init(&scd4x_handle, i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SCD4x: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Waiting 400 milliseconds to power up SCD4x...");
    vTaskDelay(pdMS_TO_TICKS(400)); 

    ESP_LOGI(TAG, "SCD4x initialized successfully");
    return ESP_OK;
}


/* Sensor task */
static void sensor_task(void *arg) {
    (void)arg;
    float temp_c = 0, hum_rh = 0;
    uint16_t co2_raw;
    bool data_ready = false;

    // Start periodic measurement (5s default)
    ESP_LOGI(TAG, " starting periodic measurement on SCD4x...");
    scd4x_start_periodic_measurement(&scd4x_handle);

    while (1) {
        // Check if new measurement is ready
        if (scd4x_get_data_ready_status(&scd4x_handle, &data_ready) == ESP_OK && data_ready) {
            // Read measurement
            if (scd4x_read_measurement(&scd4x_handle, &co2_raw, &temp_c, &hum_rh) == ESP_OK) {
            printf("CO2: %.0f ppm | Temp: %.2f°C | RH: %.2f%%\n", (float)co2_raw, temp_c, hum_rh);
        }

            // Optionally, reset data_ready flag if your sensor library allows it
            data_ready = false;
        }

        // Delay a short while before checking again
        vTaskDelay(pdMS_TO_TICKS(500));  // 0.5s polling
    }
}
\
/* Start FreeRTOS sensor task */
void start_sensor_task(void) {
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}