#include "sensors.h"
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "i2cdev.h"
#include "scd4x.h"

static const char *TAG = "sensors";
static i2c_dev_t scd4x_dev;

esp_err_t sensors_init(void) {
    // Initialize I2C subsystem (once per app)
    ESP_ERROR_CHECK(i2cdev_init());

    // Setup device descriptor
    scd4x_dev.port = I2C_NUM_0;
    scd4x_dev.addr = SCD4X_I2C_ADDR;
    /* Board: Seeed Studio XIAO ESP32C6 pinout — SDA=GPIO22, SCL=GPIO23 */
    scd4x_dev.cfg.sda_io_num = 22;
    scd4x_dev.cfg.scl_io_num = 23;
    scd4x_dev.cfg.sda_pullup_en = GPIO_PULLUP_DISABLE;
    scd4x_dev.cfg.scl_pullup_en = GPIO_PULLUP_DISABLE;
    scd4x_dev.cfg.master.clk_speed = 100000;


    /* Initialize SCD4x descriptor. Note: scd4x_init_desc() sets the device
       address and internally calls i2c_dev_create_mutex() (see scd4x driver)
       so an explicit i2c_dev_create_mutex() call is not necessary. */
    esp_err_t err = scd4x_init_desc(&scd4x_dev, scd4x_dev.port, scd4x_dev.cfg.sda_io_num, scd4x_dev.cfg.scl_io_num);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scd4x_init_desc failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Probe device presence on the bus to give a clearer message if the
       sensor is not connected or ACKing. This uses i2cdev's non-intrusive
       check which will attempt to contact the device address. */
    err = i2c_dev_check_present(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SCD4x not present on I2C bus (addr 0x%02x): %s", scd4x_dev.addr, esp_err_to_name(err));
        /* Clean up descriptor/mutex created by scd4x_init_desc() */
        scd4x_free_desc(&scd4x_dev);
        return err;
    }

    /* Follow sensor init sequence used by scd4x example: wake up, ensure in
       idle mode, reinit if needed, then start periodic measurement. This
       avoids sending start command while the sensor is sleeping or in an
       incompatible mode (which causes NACK/invalid state). */
    err = scd4x_wake_up(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scd4x_wake_up failed: %s", esp_err_to_name(err));
        /* Continue — some sensors may already be awake; fallthrough to stop */
    }

    err = scd4x_stop_periodic_measurement(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scd4x_stop_periodic_measurement failed: %s", esp_err_to_name(err));
        /* Not fatal; continue to reinit attempt */
    }

    err = scd4x_reinit(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scd4x_reinit failed: %s", esp_err_to_name(err));
        /* Continue — reinit may not be required on some modules */
    }

    /* Start periodic measurement. If this fails (NACK/invalid state), return
       the error to the caller instead of aborting via ESP_ERROR_CHECK so the
       application can handle it gracefully. */
    err = scd4x_start_periodic_measurement(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scd4x_start_periodic_measurement failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SCD4x initialized and measurement started");
    return ESP_OK;
}

esp_err_t sensors_read(float *co2_ppm, float *temperature, float *humidity) {
    /* The sensor uses periodic measurements with a 5s update interval.
       Wait up to ~5.5s for data_ready before reading. This avoids the
       previous short retry window which would frequently find no data. */
    const TickType_t timeout_ticks = pdMS_TO_TICKS(5500);
    const TickType_t poll_delay = pdMS_TO_TICKS(100);
    TickType_t start = xTaskGetTickCount();
    bool ready = false;
    esp_err_t err;

    err = i2c_dev_take_mutex(&scd4x_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to take i2c mutex: %s", esp_err_to_name(err));
        return err;
    }

    do {
        err = scd4x_get_data_ready_status(&scd4x_dev, &ready);
        if (err != ESP_OK) break;
        if (!ready) {
            vTaskDelay(poll_delay);
        }
    } while (!ready && (xTaskGetTickCount() - start) < timeout_ticks);

    if (!ready) {
        i2c_dev_give_mutex(&scd4x_dev);
        ESP_LOGW(TAG, "Data not ready after %u ms", (unsigned int)pdTICKS_TO_MS(timeout_ticks));
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t co2_raw = 0;
    err = scd4x_read_measurement(&scd4x_dev, &co2_raw, temperature, humidity);
    i2c_dev_give_mutex(&scd4x_dev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement: %d", err);
        return err;
    }

    *co2_ppm = (float)co2_raw;
    ESP_LOGI(TAG, "CO2: %.0f ppm, Temp: %.2f °C, Humidity: %.2f %%", *co2_ppm, *temperature, *humidity);
    return ESP_OK;
}

void sensors_deinit(void) {
    // Stop periodic measurement and free descriptor (scd4x_free_desc deletes the mutex)
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&scd4x_dev));
    ESP_ERROR_CHECK(scd4x_free_desc(&scd4x_dev));

    // Deinit I2C subsystem
    ESP_ERROR_CHECK(i2cdev_done());
}

static void sensor_task(void *arg) {
    (void)arg;
    float co2, temp, hum;

    while (1) {
        if (sensors_read(&co2, &temp, &hum) == ESP_OK) {
            printf("CO2: %.0f ppm, Temp: %.2f C, Humidity: %.2f %%\n", co2, temp, hum);
        }
        /* Periodic measurements update every 5s, match that here to avoid
           unnecessary polling */
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5s delay
    }
}

void start_sensor_task(void) {
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
