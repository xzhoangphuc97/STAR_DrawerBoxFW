
idf_component_register(
    SRCS "cui.c"
    INCLUDE_DIRS "."
    REQUIRES "freertos"  "Ethernet" "BLE" "Wifi" "USB_CDC"
)