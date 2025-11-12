#include "openthread_manager.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_openthread.h"
#include "openthread/thread.h"
#include "openthread/joiner.h"

static const char *TAG = "OPENTHREAD";
#define JOINER_PSKD "LORETDALYPSKD"

static void ot_state_cb(uint32_t flags, void *context)
{
    otInstance *instance = (otInstance *)context;

    if (flags & OT_CHANGED_THREAD_ROLE) {
        otDeviceRole role = otThreadGetDeviceRole(instance);
        ESP_LOGI(TAG, "Thread role changed: %d", role);
    }
}

esp_err_t ot_manager_init(void)
{
    // Platform config
    esp_openthread_platform_config_t platform_config = {
        .radio_config = {.radio_mode = RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = HOST_CONNECTION_MODE_NONE},
        .port_config = {
            .storage_partition_name = "nvs",
            .netif_queue_size = 20,
            .task_queue_size = 20,
        },
    };

    ESP_ERROR_CHECK(esp_openthread_init(&platform_config));

    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGE(TAG, "Failed to get OpenThread instance");
        return ESP_FAIL;
    }

    // Register native OpenThread state changed callback
    otSetStateChangedCallback(instance, ot_state_cb, instance);

    ESP_LOGI(TAG, "OpenThread platform initialized");
    return ESP_OK;
}


static void joiner_cb(otError error, void *ctx)
{
    if (error == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "Joiner successfully joined the Thread network");
    } else {
        ESP_LOGE(TAG, "Joiner failed: %d", error);
    }
}

void ot_manager_start_joiner(void)
{
    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGE(TAG, "Cannot start joiner: OT instance is NULL");
        return;
    }

    const char *vendor_name      = "ESP";
    const char *vendor_model     = "CO2Sensor";
    const char *vendor_sw_ver    = "1.0";
    const char *vendor_data      = NULL;
    const char *provisioning_url = NULL;

    otError err = otJoinerStart(
        instance,
        JOINER_PSKD,
        provisioning_url,
        vendor_name,
        vendor_model,
        vendor_sw_ver,
        vendor_data,
        joiner_cb,
        NULL
    );

    if (err == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "Joiner started with PSKd: %s", JOINER_PSKD);
    } else {
        ESP_LOGE(TAG, "Failed to start joiner: %d", err);
    }
}