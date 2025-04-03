idf_component_register(
    SRCS
    "esp_zb_temperature_sensor.c"
    "sgp30.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_common freertos nvs_flash esp_system
)
