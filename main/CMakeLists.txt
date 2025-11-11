idf_component_register(
    SRCS "main.c" "app_main.c" "openthread.c" "sensors.c"
    INCLUDE_DIRS "."
    REQUIRES openthread nvs_flash esp_netif vfs esp_driver_i2c scd4x
)

# Optional: force target
set(IDF_TARGET "esp32c6")

