#include "openthread_manager.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_netif.h"
#include "openthread/thread.h"
#include "openthread/joiner.h"
#include "esp_openthread_netif_glue.h"

static const char *TAG = "OPENTHREAD";
#define JOINER_PSKD "J01NME"

static void ot_state_cb(uint32_t flags, void *context)
{
    otInstance *instance = (otInstance *)context;

    if (flags & OT_CHANGED_THREAD_ROLE) {
        otDeviceRole role = otThreadGetDeviceRole(instance);
        ESP_LOGI(TAG, "Thread role changed: %d", role);

        switch (role) {
            case OT_DEVICE_ROLE_DISABLED:
                ESP_LOGW(TAG, "Thread stack disabled");
                break;
            case OT_DEVICE_ROLE_DETACHED:
                ESP_LOGW(TAG, "Node detached from Thread network");
                break;
            case OT_DEVICE_ROLE_CHILD:
                ESP_LOGI(TAG, "Node joined as CHILD");
                break;
            case OT_DEVICE_ROLE_ROUTER:
                ESP_LOGI(TAG, "Node is a ROUTER");
                break;
            case OT_DEVICE_ROLE_LEADER:
                ESP_LOGI(TAG, "Node is the LEADER");
                break;
        }
    }

    if (flags & OT_CHANGED_THREAD_PARTITION_ID) {
        ESP_LOGI(TAG, "Thread partition ID changed");
    }

    if (flags & OT_CHANGED_THREAD_NETDATA) {
        ESP_LOGI(TAG, "Thread network data updated");
    }
}


esp_err_t ot_manager_init(void)
{
    esp_openthread_platform_config_t platform_config = {
        .radio_config = {.radio_mode = RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = HOST_CONNECTION_MODE_NONE},
        .port_config = {
            .storage_partition_name = "nvs",
            .netif_queue_size = 20,
            .task_queue_size = 20,
        },
    };
    esp_netif_config_t config = ESP_NETIF_DEFAULT_OPENTHREAD();

    ESP_ERROR_CHECK(esp_openthread_init(&platform_config));

    esp_netif_t *esp_netif = esp_netif_new(&config);
    esp_netif_driver_base_t *glue = esp_openthread_netif_glue_init(&platform_config);
    ESP_ERROR_CHECK(esp_netif_attach(esp_netif, glue));

    otInstance *instance = esp_openthread_get_instance();
    if (instance == NULL) {
        ESP_LOGE(TAG, "Failed to get OpenThread instance");
        return ESP_FAIL;
    }


    // Register OpenThread state changed callback
    otSetStateChangedCallback(instance, ot_state_cb, instance);

    ESP_LOGI(TAG, "OpenThread platform initialized");
    return ESP_OK;
}

void openthread_mainloop_task(void *arg)
{
    esp_openthread_launch_mainloop();
}

void start_openThread(void){
    ESP_LOGI(TAG, "Bringing up OpenThread interface...");

    otInstance *instance = esp_openthread_get_instance();
    assert(instance);

    if (otIp6SetEnabled(instance, true) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable IPv6");
    }

    if (otThreadSetEnabled(instance, true) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to start Thread protocol");
    }
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