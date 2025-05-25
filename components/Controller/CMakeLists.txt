idf_component_register(SRCS "Controller.c" "Drawer4ConnSensor.c" "Drawer3ConnSensor.c" "Drawer2ConnSensor.c"  "Drawer1ConnSensor.c" "defines.c" "CompulsionSw1.c" "CompulsionSw2.c" "Drawer1.c" "Drawer2.c" "Drawer3.c" "Drawer4.c" 
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_adc )
