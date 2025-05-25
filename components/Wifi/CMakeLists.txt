idf_component_register(SRCS "wifi.c" 
                       INCLUDE_DIRS "."
                       REQUIRES "esp_wifi" "esp_netif" "lwip" "CUI" "nvs_flash"  "freertos"
                      )