#include <stdio.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "openthread/error.h"
#include "openthread/thread.h"
#include "esp_heap_caps.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sample-implementations/esp32/sensirion_i2c_esp32_config.h"
#include "scd4x_i2c.h"

static const char *TAG = "CO2_SENSOR_OT";

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   22    // changed to SDA GPIO22
#define I2C_MASTER_SCL_IO   23    // changed to SCL GPIO23
#define I2C_MASTER_FREQ_HZ  100000
#define SCD41_ADDR 0x62

#define SENSOR_TASK_STACK_BYTES   (8 * 1024)
#define SENSOR_TASK_PRIORITY      5

static void log_heap(const char *prefix)
{
    ESP_LOGI(TAG, "%s free heap: %u", prefix, (unsigned)esp_get_free_heap_size());
}

static esp_err_t i2c_init(void)
{
    // If a driver exists, delete it first to avoid INVALID_STATE errors
    i2c_driver_delete(I2C_MASTER_NUM);

    // Try to recover bus by toggling SCL if SDA is stuck low
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SCL_IO) | (1ULL << I2C_MASTER_SDA_IO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // If SDA held low, pulse SCL up to 9 times to clock out the stuck slave
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    if (gpio_get_level(I2C_MASTER_SDA_IO) == 0) {
        ESP_LOGW(TAG, "SDA held low — attempting clock pulses to free bus");
        for (int i = 0; i < 9 && gpio_get_level(I2C_MASTER_SDA_IO) == 0; ++i) {
            gpio_set_level(I2C_MASTER_SCL_IO, 0);
            ets_delay_us(5);
            gpio_set_level(I2C_MASTER_SCL_IO, 1);
            ets_delay_us(5);
        }
        // send STOP: SDA low -> SCL high -> SDA high
        gpio_set_level(I2C_MASTER_SDA_IO, 0);
        ets_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        ets_delay_us(5);
        gpio_set_level(I2C_MASTER_SDA_IO, 1);
        ets_delay_us(5);
        ESP_LOGI(TAG, "Bus recovery sequence complete, SDA=%d", gpio_get_level(I2C_MASTER_SDA_IO));
    }

    // Configure normal I2C driver
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Configure Sensirion ESP32 HAL and initialize it so scd4x driver
       can use the higher-level abstraction. This relies on the sample
       implementation included in components/scd4x/sample-implementations/esp32. */
    struct esp32_i2c_config i2c_cfg = {
        .freq = I2C_MASTER_FREQ_HZ,
        .addr = SCD41_ADDR,
        .port = I2C_MASTER_NUM,
        .sda = I2C_MASTER_SDA_IO,
        .scl = I2C_MASTER_SCL_IO,
        .sda_pullup = true,
        .scl_pullup = true,
    };

    if (sensirion_i2c_config_esp32(&i2c_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "sensirion_i2c_config_esp32 failed");
    } else {
        /* Initialize the Sensirion HAL - this will create the internal i2c_dev
           mutex and print a log on success/failure. */
        sensirion_i2c_hal_init();
        if (sensirion_i2c_esp32_ok() != ESP_OK) {
            ESP_LOGW(TAG, "Sensirion I2C HAL reported error after init");
        }
    }

    ESP_LOGI(TAG, "I2C initialized SDA=%d SCL=%d @%uHz", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    // small settle
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

/* Simple I2C bus scanner used for debug: logs any address that ACKs. */
static void i2c_bus_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus %d...", I2C_MASTER_NUM);
    bool found = false;
    for (int addr = 1; addr < 0x7F; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (res == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
            found = true;
        } else if (res == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Addr 0x%02X: timeout (check pull-ups/power)", addr);
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "No I2C devices found. Verify wiring, power, pull-ups.");
    } else {
        ESP_LOGI(TAG, "I2C scan completed");
    }
}


static esp_err_t scd41_start_measurement(void)
{
    /* Use scd4x driver abstraction */
    scd4x_init(SCD41_ADDR);
    int16_t r = scd4x_start_periodic_measurement();
    if (r == 0) {
        ESP_LOGI(TAG, "SCD41 start successful");
        return ESP_OK;
    }
    ESP_LOGW(TAG, "SCD41 start failed (driver): %d", r);
    return ESP_FAIL;
}



/* Sensirion CRC8 check (polynomial 0x31, init 0xFF) */
static bool scd41_check_crc(const uint8_t *bytes)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; ++i) {
        crc ^= bytes[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return (crc == bytes[2]);
}

static esp_err_t scd41_read_measurement(uint16_t *co2, float *temp, float *rh)
{
    uint16_t raw_temp = 0;
    uint16_t raw_rh = 0;
    int32_t temp_mdeg = 0;
    int32_t rh_mpermil = 0;
    int16_t r = scd4x_read_measurement(co2, &temp_mdeg, &rh_mpermil);
    if (r != 0) {
        ESP_LOGW(TAG, "scd4x_read_measurement failed: %d", r);
        return ESP_FAIL;
    }

    *temp = ((float)temp_mdeg) / 1000.0f;
    *rh = ((float)rh_mpermil) / 1000.0f;
    return ESP_OK;
}


static void sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor task starting");

    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed, deleting sensor task");
        vTaskDelete(NULL);
        return;
    }

    // Quick scan for debugging
    i2c_bus_scan();

    if (scd41_start_measurement() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SCD41 measurement. Check wiring, VDD/GND, pull-ups.");
        vTaskDelay(pdMS_TO_TICKS(2000));
        // continue - attempts to read will trigger re-init logic below
    } else {
        vTaskDelay(pdMS_TO_TICKS(200)); // give sensor time to start
    }

    uint32_t consecutive_failures = 0;

    while (1) {
        uint16_t co2;
        float t, h;
        esp_err_t err = scd41_read_measurement(&co2, &t, &h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "CO2: %u ppm  Temp: %.2f C  RH: %.2f %%", co2, t, h);
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
            ESP_LOGW(TAG, "SCD41 read failed (%u): %s", consecutive_failures, esp_err_to_name(err));

            // On bus errors, attempt driver reset + re-init + restart sensor
            if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_CRC) {
                ESP_LOGW(TAG, "Attempting I2C driver reset & sensor restart");
                i2c_driver_delete(I2C_MASTER_NUM);
                vTaskDelay(pdMS_TO_TICKS(50));
                if (i2c_init() == ESP_OK) {
                    // Try to start measurement again but don't block forever
                    if (scd41_start_measurement() == ESP_OK) {
                        ESP_LOGI(TAG, "Recovered SCD41 after reset");
                        consecutive_failures = 0;
                    } else {
                        ESP_LOGW(TAG, "SCD41 restart failed after reset");
                    }
                } else {
                    ESP_LOGW(TAG, "I2C re-init failed during recovery");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}



/*************************************************************
 * THREAD CALLBACK (state change)
 *************************************************************/
static void ot_state_cb(otChangedFlags flags, void *ctx)
{
    if (flags & OT_CHANGED_THREAD_ROLE) {
        otInstance *instance = (otInstance *)ctx;
        otDeviceRole role = otThreadGetDeviceRole(instance);
        ESP_LOGI(TAG, "Thread role changed: %d", role);
    }
}

/*************************************************************
 * OPTIONAL JOINER
 *************************************************************/
#define JOINER_PSKD "LORETDALYPSKD"

static void start_joiner(void)
{
    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGW(TAG, "No OpenThread instance available");
        return;
    }

    /* Joiner PSKd for commissioning (not auto-started by default).
       To start joiner from CLI: joiner start LORETDALYPSKD
       Or uncomment start_joiner() call in app_main(). */
    ESP_LOGI(TAG, "Joiner PSKd: %s (use CLI or call start_joiner() to begin)", JOINER_PSKD);
}

/*************************************************************
 * MAIN APPLICATION
 *************************************************************/
void app_main(void)
{
    ESP_LOGI(TAG, "Starting CO2 Sensor + OpenThread");
    log_heap("startup");

    /* Initialize NVS (required by OpenThread) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* Initialize event loop and netif */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "Event loop and netif initialized");

    /* Register VFS eventfd (required by OpenThread task queue) */
    esp_vfs_eventfd_config_t eventfd_cfg = {
        .max_fds = 3,
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_cfg));
    ESP_LOGI(TAG, "VFS eventfd registered");

    /* NOTE: Do NOT call esp_openthread_lock_init() here—
       esp_openthread_init() handles lock initialization internally. */

    /* Initialize OpenThread platform with native radio */
    esp_openthread_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = HOST_CONNECTION_MODE_NONE,
        },
        .port_config = {
            .storage_partition_name = "nvs",
            .netif_queue_size = 20,
            .task_queue_size = 20,
        },
    };

    ESP_ERROR_CHECK(esp_openthread_init(&platform_config));
    ESP_LOGI(TAG, "OpenThread platform initialized");


    /* Start sensor task */
    if (xTaskCreate(sensor_task, "sensor_task",
                    SENSOR_TASK_STACK_BYTES, NULL,
                    SENSOR_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }
    ESP_LOGI(TAG, "Sensor task created");

    /* Optionally start joiner (uncomment to enable) */
    // start_joiner();

    log_heap("before launch_mainloop");

    /* Launch OpenThread mainloop (blocks) */
    ESP_LOGI(TAG, "Launching OpenThread mainloop...");
    esp_openthread_launch_mainloop();

    /* Should not reach here */
    ESP_LOGE(TAG, "OpenThread mainloop returned unexpectedly");
}
