#include "Drawer3.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "esp_log.h"

static const char *TAGd = "Drawer3";

static bool isDrawer3Initialized = false; // Initialization flag

void InitDrawer3(void)
{
    gpio_reset_pin(DK3_CNT);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DK3_CNT),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE      // Disable pull-up

    };
    gpio_config(&io_conf);
    isDrawer3Initialized = true;
}

ER_T DriveDrawer3(uint16_t onTime, uint16_t offTime)
{
    if (!isDrawer3Initialized)
    {
        return ER_OBJ; // Illegal processing if not initialized
    }
    if (onTime < 10 || onTime > 10000 || offTime < 10 || offTime > 10000)
    {
        return ER_PAR; // Parameter out of range
    }
    ESP_LOGI(TAGd, "Drawer 3 will drive: ON for %d ms, OFF for %d ms.", onTime, offTime);
    gpio_set_level(DK3_CNT, 1);
    vTaskDelay(onTime / portTICK_PERIOD_MS);

    gpio_set_level(DK3_CNT, 0);
    vTaskDelay(offTime / portTICK_PERIOD_MS);
    return ER_OK;
}

void OffDrawer3(void)
{
    gpio_set_level(DK3_CNT, 0);
    ESP_LOGE(TAGd, "Force Stop Drawer 3 Run!");
}
