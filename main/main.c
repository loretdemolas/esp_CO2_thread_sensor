#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"

#include "sensors.h"
#include "openthread_manager.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    esp_vfs_eventfd_config_t cfg = {.max_fds = 3};
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&cfg));

    ESP_ERROR_CHECK(ot_manager_init());\
    ot_manager_start_joiner();

    if (sensors_init() == ESP_OK) {
        start_sensor_task();
    }

    ESP_LOGI(TAG, "OpenThread is running (native C6 mode).");
}