idf_component_register(SRCS "ethernet.c"
                      INCLUDE_DIRS "."
                      REQUIRES "driver" "freertos" "CUI"
                     )