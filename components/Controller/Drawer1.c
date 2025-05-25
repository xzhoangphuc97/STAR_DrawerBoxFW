#include "Drawer1.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"

static const char *TAGd = "Drawer1";

static bool isDrawer1Initialized = false; // Initialization flag

void InitDrawer1(void)
{
    gpio_reset_pin(DK1_CNT);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DK1_CNT),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE      // Disable pull-up

    };
    gpio_config(&io_conf);
    isDrawer1Initialized = true;
}

// Function to drive Drawer 1
ER_T DriveDrawer1(uint16_t onTime, uint16_t offTime)
{
    if (!isDrawer1Initialized)
    {
        return ER_OBJ; // Illegal processing if not initialized
    }
    if (onTime < 10 || onTime > 10000 || offTime < 10 || offTime > 10000)
    {
        return ER_PAR; // Parameter out of range
    }
    ESP_LOGI(TAGd, "Drawer 1 will drive: ON for %d ms, OFF for %d ms.", onTime, offTime);
    // Generate ON pulse
    gpio_set_level(DK1_CNT, 1);               // Set GPIO pin HIGH
    vTaskDelay(onTime / portTICK_PERIOD_MS);  // Wait for ON time in milliseconds
    gpio_set_level(DK1_CNT, 0);               // Set GPIO pin LOW
    vTaskDelay(offTime / portTICK_PERIOD_MS); // Wait for OFF time in milliseconds
    return ER_OK;                             // Successfully driven
}

// Function to forcibly turn off Drawer 1
void OffDrawer1(void)
{
    gpio_set_level(DK1_CNT, 0); // Set GPIO pin to LOW (OFF)
    ESP_LOGE(TAGd, "Force Stop Drawer 1 Run!");
}
