#include <stdio.h>
#include "../components/Controller/controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble.h"
#include "ethernet.h"
#include "wifi.h"
#include "usb_cdc.h"
#include "cui.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <string.h>
#include <esp_random.h>
#include "../components/defines.h"


// Logging tag for main
static const char *TAGm = "MAIN";
void InitialSystem(void);


//------------- [CONTROLLER BLOCK] CMT OUT FOR DEBUG --------------
void subTask(void *pvParameter)
{
    while (1)
    {
        ForceStopDrawer(VoltageMonitor());
        GetASBStatus();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mainTask(void)
{
    uint8_t data[DATA_SIZE];
    size_t len;
    char processed_data[DATA_SIZE];
    char *arr[32];
    size_t arr_len;
    uint16_t read_idx = 0;

    while (1)
    {
        if (!common_buffer_is_empty())
        {
            len = DATA_SIZE - 1;
            if (common_buffer_read(data, &len) == ESP_OK)
            {
                read_idx = (read_idx + 1) % BUFFER_SIZE;

                // Main Step 3: Check for ESC/ACK/SOH control sequences
                // if (cui_check_bit_esc_ack_soh(data, len))
                if (cui_check_esc_ack_soh(data, len))
                {
                    // Main Step 3.1:Get ASB status when have request
                    ESP_LOGI(TAG, "[ESC ACK SOH]: %s", data);
                    GetASBStatus();
                    uint8_t s1 = GetABS_S1();
                    uint8_t s2 = GetABS_S2();
                    uint8_t s3 = GetABS_S3();
                    uint8_t s4 = GetABS_S4();
                    send_asb(s1, s2, s3, s4);
                }
                else
                {
                    // Dummy
                    cui_remove_multiple_spaces((char *)data, processed_data, DATA_SIZE);
                    cui_string_to_array(processed_data, arr, &arr_len, 32);
                    cui_array_to_string((const char **)arr, arr_len, processed_data, DATA_SIZE);
                    ESP_LOGI(TAG, "Processed data: %s", processed_data);
                    for (size_t i = 0; i < arr_len; i++)
                    {
                        free(arr[i]);
                    }

                    //------------- [CONTROLLER BLOCK] CMT OUT FOR DEBUG  --------------
                    // Main Step 4: Check for another Commnand
                    // delspace((char *)data, processed_data, DATA_SIZE);
                    ExecuteCommand((char *)data);
                    ESP_LOGI(TAGm, "End Drawer Process...");
                }
            }
        }
        if (getChangeFlagStatus() == true && getASBFlag() == true)
        {
            uint8_t s1 = GetABS_S1();
            uint8_t s2 = GetABS_S2();
            uint8_t s3 = GetABS_S3();
            uint8_t s4 = GetABS_S4();
            send_asb(s1, s2, s3, s4);
            setChangeFlagStatusToFalse();
        }

        vTaskDelay(pdMS_TO_TICKS(TaskDelayTime)); // Maximizes power savings for battery-operated devices
    }
}
void app_main(void)
{
    // Initialize and run main application
    // Step 1: Log application start
    ESP_LOGI(TAGm, "Starting application");
    // Step 2: Initialize NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAGm, "NVS partition issue detected, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAGm, "NVS initialized");
    InitADCChannels();
    // Start system operation
    // Main Step 1: Initial processing, Sampling conditions, Polarity settings
    InitialSystem();

    // Main Step 2: Enable communication interfaces
    ethernet_enable();
    ESP_LOGI(TAGm, "LAN enabled");
    wifi_enable();
    ESP_LOGI(TAGm, "WiFi enabled");
    usb_cdc_enable();
    ESP_LOGI(TAGm, "USB CDC enabled");
    ble_enable();
    ESP_LOGI(TAGm, "BLE enabled");

    //------------- [CONTROLLER BLOCK] CMT OUT FOR DEBUG --------------
    // // Create subTask to run in parallel
    xTaskCreate(subTask, "SubTask", 2048, NULL, 5, NULL);

    ESP_LOGI(TAGm, "Main task running...");
    // xTaskCreate(mainTask, "MainTask", 2048, NULL, 5, NULL);
    mainTask();
}
void InitialSystem(void)
{
    ESP_LOGI(TAGm, "System Start initializing....");
    // Step 1: set up GPIO
    InitLed();
    ConfigureGpioUnsed();
    InitPowerOut();
    InitSupply();
    // Step 2: Set VM-EM to HIGH
    SetPowerOut();
    // Step 3: Set up GPIO for cash drawer
    InitDrawer1();
    InitDrawer2();
    InitDrawer3();
    InitDrawer4();
    // Step 4: Set up GPIO for Switch
    InitCompulsionSw1();
    InitCompulsionSw2();
    // Step 5: Set up GPIO for Connection Sensor Drawer
    InitDrawer1ConnSensor();
    InitDrawer2ConnSensor();
    InitDrawer3ConnSensor();
    InitDrawer4ConnSensor();
    // Step 6: Set up communication Interface
    ethernet_init();
    ESP_LOGI(TAGm, "LAN initialized");
    wifi_init();
    ESP_LOGI(TAGm, "WiFi initialized");
    usb_cdc_init();
    ESP_LOGI(TAGm, "USB CDC initialized");
    ble_init();
    ESP_LOGI(TAGm, "BLE initialized");
    cui_init();
    ESP_LOGI(TAGm, "CUI initialized");
    ESP_LOGI(TAGm, "System GPIO Finish initialized!");
    // Step 7: Set onCount and offCount for sampling Switch
    StartCompulsionSw1Sampling(ONCountSW1, OFFCountSW1);
    StartCompulsionSw2Sampling(ONCountSW2, OFFCountSW2);
    // Step 8: Set onCount and offCount for sampling Connection Sensor
    StartDrawer1ConnSensorSampling(ONCountConnSensor1, OFFCountConnSensor1);
    StartDrawer2ConnSensorSampling(ONCountConnSensor2, OFFCountConnSensor2);
    StartDrawer3ConnSensorSampling(ONCountConnSensor3, OFFCountConnSensor3);
    StartDrawer4ConnSensorSampling(ONCountConnSensor4, OFFCountConnSensor4);
    // Step 9: set up Polarity for Switch
    SetupCompulsionSw1Polarity(SW1Polarity);
    SetupCompulsionSw2Polarity(SW2Polarity);
    // Step 10: set up Polarity for Connect Sensor
    SetupDrawer1ConnSensorPolarity(Drawer1Polarity);
    SetupDrawer2ConnSensorPolarity(Drawer2Polarity);
    SetupDrawer3ConnSensorPolarity(Drawer3Polarity);
    SetupDrawer4ConnSensorPolarity(Drawer4Polarity);
    ESP_LOGI(TAGm, "System Finish initialized!");
}