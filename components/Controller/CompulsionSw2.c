#include "CompulsionSw2.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Pin and polarity settings

static BYTE sw2Polarity = 1; // Default: HIGH = pressed
static const char *TAG = "CompulsionSW2";

static WORD onCounterSw2 = 0, offCounterSw2 = 0;
static bool isSW2Initialized = false; // Initialization flag

void InitCompulsionSw2(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DK_SW2),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    isSW2Initialized = true;

    // ESP_LOGI(TAG, "CompulsionSW1 initialized (GPIO %d)", DK_SW1);
}

ER_T StartCompulsionSw2Sampling(WORD onCnt, WORD offCnt)
{
    if (onCnt < 1 || onCnt > 65535 || offCnt < 1 || offCnt > 65535)
    {
        return ER_PAR; // Invalid input range
    }
    onCounterSw2 = onCnt;
    offCounterSw2 = offCnt;
    return ER_OK;
}

void StopCompulsionSw2Sampling(void)
{
    ESP_LOGI(TAG, "CompulsionSW2 sampling stopped.");
}

void CompulsionSw2Handler(void)
{
    ESP_LOGD(TAG, "CompulsionSW2 handler invoked for chattering removal.");
    int countLow = 0, countHigh = 0;
    int current_value;
    do
    {
        current_value = gpio_get_level(DK_SW2);
        if (current_value == 0)
        { // LOW state
            countLow++;
            countHigh = 0; // Reset high counter
        }
        else
        { // HIGH state
            countHigh++;
            countLow = 0; // Reset low counter
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for stability (adjust as needed)

    } while (countLow < onCounterSw2 && countHigh < offCounterSw2);
}

void SetupCompulsionSw2Polarity(BYTE pole)
{
    sw2Polarity = pole;
    ESP_LOGI(TAG, "CompulsionSW1 polarity set to %s", pole ? "HIGH" : "LOW");
}

bool IsCompulsionSw2Pushed(void)
{
    CompulsionSw2Handler();
    int currentState = gpio_get_level(DK_SW2);
    if (sw2Polarity == 0)
    {
        return !currentState;
    }
    else
    {
        return currentState;
    }
}
