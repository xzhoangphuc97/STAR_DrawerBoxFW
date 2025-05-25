#include "Drawer2ConnSensor.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Drawer2ConnSensor";

static adc_oneshot_unit_handle_t adc1_handle;

#define DK2_SNS_CHANNEL ADC_CHANNEL_1

static BYTE drawer2Polarity = 1; // Default polarity: HIGH = Connected
static WORD onConnSensor2 = 0;
static WORD offConnSensor2 = 0;
static bool isConnSensor2Initialized = false; // Initialization flag
void InitDrawer2ADC(adc_oneshot_unit_handle_t adc_handle)
{
    adc1_handle = adc_handle;
}
void InitDrawer2ConnSensor(void)
{

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_CHANNEL1, // Adjust attenuation as needed
    };
    adc_oneshot_config_channel(adc1_handle, DK2_SNS_CHANNEL, &config);

    isConnSensor2Initialized = true;
    ESP_LOGI(TAG, "GPIO[%d] Analog input initialized", DK2_SNS_CHANNEL);
}

ER_T StartDrawer2ConnSensorSampling(WORD onCnt, WORD offCnt)
{
    if (onCnt < 1 || onCnt > 65535 || offCnt < 1 || offCnt > 65535)
    {
        ESP_LOGE(TAG, "Invalid arguments: onCnt=%d, offCnt=%d", onCnt, offCnt);
        return ER_PAR;
    }
    onConnSensor2 = onCnt;
    offConnSensor2 = offCnt;
    return ER_OK;
}

void StopDrawer2ConnSensorSampling(void)
{
    ESP_LOGI(TAG, "Drawer2 connection sensor sampling stopped.");
}

void Drawer2ConnSensorHandler(void)
{
    ESP_LOGD(TAG, "Drawer2 connection sensor handler invoked for chattering removal.");

    int count_low = 0, count_high = 0;
    int current_value;

    do
    {
        int adc_raw;
        adc_oneshot_read(adc1_handle, DK2_SNS_CHANNEL, &adc_raw);      // Read ADC value
        float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL1) * V_REF; // Convert to voltage
        current_value = (voltage > LOGICVOL_DRAWER2) ? 1 : 0;

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

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for stability (adjust as needed)

    } while (count_low < onConnSensor2 && count_high < offConnSensor2);
}

void SetupDrawer2ConnSensorPolarity(BYTE pole)
{
    drawer2Polarity = pole;
    ESP_LOGI(TAG, "Drawer2 connection sensor polarity set to %s", pole ? "HIGH" : "LOW");
}

bool IsConnectedDrawer2(void)
{
    Drawer2ConnSensorHandler();
    int adc_raw;
    adc_oneshot_read(adc1_handle, DK2_SNS_CHANNEL, &adc_raw);      // Read ADC value
    float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL1) * V_REF; // Convert to voltage
    // ESP_LOGI(TAG, "Drawer2 connection sensor voltage: %f", voltage);
    bool currentValue = (voltage > LOGICVOL_DRAWER1) ? 1 : 0;

    if (drawer2Polarity == 0)
    {
        return !currentValue;
    }
    else
    {
        return currentValue;
    }
}
