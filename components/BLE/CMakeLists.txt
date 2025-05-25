
idf_component_register(
    SRCS "ble.c"
    INCLUDE_DIRS "."
    REQUIRES "bt" "freertos" "CUI"
    PRIV_REQUIRES "esp_wifi"
)