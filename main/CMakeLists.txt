idf_component_register(
    SRCS 
        "main.c"
        "sensors.c"
        "openthread_manager.c"
        "scd4x.c"
    INCLUDE_DIRS "." 
    REQUIRES
        openthread
        nvs_flash
        esp_netif
        vfs
        driver
)
