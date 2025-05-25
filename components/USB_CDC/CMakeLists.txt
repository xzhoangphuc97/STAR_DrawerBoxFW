idf_component_register(
    SRCS "usb_cdc.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_tinyusb" "freertos" "CUI"
)