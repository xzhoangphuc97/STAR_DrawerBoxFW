#include "Drawer2.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"

static const char *TAGd = "Drawer2";

static bool isDrawer2Initialized = false; // Initialization flag

void InitDrawer2(void)
{
    gpio_reset_pin(DK2_CNT);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DK2_CNT),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE      // Disable pull-up

    };
    gpio_config(&io_conf);
    isDrawer2Initialized = true;
}

ER_T DriveDrawer2(uint16_t onTime, uint16_t offTime)
{
    if (!isDrawer2Initialized)
    {
        return ER_OBJ; // Illegal processing if not initialized
    }
    if (onTime < 10 || onTime > 10000 || offTime < 10 || offTime > 10000)
    {
        return ER_PAR; // Parameter out of range
    }
    ESP_LOGI(TAGd, "Drawer 2 will drive: ON for %d ms, OFF for %d ms.", onTime, offTime);

    gpio_set_level(DK2_CNT, 1);
    vTaskDelay(onTime / portTICK_PERIOD_MS);

    gpio_set_level(DK2_CNT, 0);
    vTaskDelay(offTime / portTICK_PERIOD_MS);
    return ER_OK;
}

void OffDrawer2(void)
{
    gpio_set_level(DK2_CNT, 0);
    ESP_LOGE(TAGd, "Force Stop Drawer 2 Run!");
}
