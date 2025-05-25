#include "CompulsionSw1.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// Pin and polarity settings

static BYTE sw1Polarity = 1; // Default: HIGH = pressed
static const char *TAG = "CompulsionSW1";

static WORD onCounterSw1 = 0, offCounterSw1 = 0;
static bool isSW1Initialized = false; // Initialization flag

void InitCompulsionSw1(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DK_SW1),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    isSW1Initialized = true;
    // ESP_LOGI(TAG, "CompulsionSW1 initialized (GPIO %d)", DK_SW1);
}

ER_T StartCompulsionSw1Sampling(WORD onCnt, WORD offCnt)
{
    if (onCnt < 1 || onCnt > 65535 || offCnt < 1 || offCnt > 65535)
    {
        return ER_PAR; // Invalid input range
    }
    onCounterSw1 = onCnt;
    offCounterSw1 = offCnt;
    return ER_OK;
}

void StopCompulsionSw1Sampling(void)
{
    ESP_LOGI(TAG, "CompulsionSW1 sampling stopped.");
}

void CompulsionSw1Handler(void)
{
    ESP_LOGD(TAG, "CompulsionSW1 handler invoked for chattering removal.");
    int count_low = 0, count_high = 0;
    int current_value;
    do
    {
        current_value = gpio_get_level(DK_SW1);
        if (current_value == 0)
        { // LOW state
            count_low++;
            count_high = 0; // Reset high counter
        }
        else
        { // HIGH state
            count_high++;
            count_low = 0; // Reset low counter
        }
        // uint64_t time_ms = esp_timer_get_time() / 1000;  // Convert microseconds to milliseconds
        // printf("Current time: %llu ms\n", time_ms);
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for stability (adjust as needed)

    } while (count_low < onCounterSw1 && count_high < offCounterSw1);
}

void SetupCompulsionSw1Polarity(BYTE pole)
{
    sw1Polarity = pole;
    ESP_LOGI(TAG, "CompulsionSW1 polarity set to %s", pole ? "HIGH" : "LOW");
}

bool IsCompulsionSw1Pushed(void)
{
    CompulsionSw1Handler();
    bool currentState = gpio_get_level(DK_SW1);
    if (sw1Polarity == 0)
    {
        return !currentState;
    }
    else
    {
        return currentState;
    }
}
