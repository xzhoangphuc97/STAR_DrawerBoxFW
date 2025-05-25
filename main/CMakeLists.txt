idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES "BLE" "Ethernet" "Wifi" "USB_CDC" "CUI" "Controller"
                     )