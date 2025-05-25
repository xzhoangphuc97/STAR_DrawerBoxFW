#include "Drawer1ConnSensor.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Drawer1ConnSensor";
static adc_oneshot_unit_handle_t adc1_handle;

static BYTE drawer1Polarity = 1; // Default polarity: HIGH = Connected
static WORD onConnSensor1 = 0;
static WORD offConnSensor1 = 0;
static bool isConnSensor1Initialized = false; // Initialization flag

#define DK1_SNS_CHANNEL ADC_CHANNEL_0
void InitDrawer1ADC(adc_oneshot_unit_handle_t adc_handle)
{
    adc1_handle = adc_handle;
}
void InitDrawer1ConnSensor(void)
{
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_CHANNEL0, // Adjust attenuation as needed
    };
    adc_oneshot_config_channel(adc1_handle, DK1_SNS_CHANNEL, &config);

    isConnSensor1Initialized = true;
    ESP_LOGI(TAG, "GPIO[%d] Analog input initialized using", DK1_SNS_CHANNEL);
}

ER_T StartDrawer1ConnSensorSampling(WORD onCnt, WORD offCnt)
{
    if (onCnt < 1 || onCnt > 65535 || offCnt < 1 || offCnt > 65535)
    {
        ESP_LOGE(TAG, "Invalid arguments: onCnt=%d, offCnt=%d", onCnt, offCnt);
        return ER_PAR;
    }
    onConnSensor1 = onCnt;
    offConnSensor1 = offCnt;
    return ER_OK;
}

void StopDrawer1ConnSensorSampling(void)
{
    ESP_LOGI(TAG, "Drawer1 connection sensor sampling stopped.");
}

void Drawer1ConnSensorHandler(void)
{
    ESP_LOGD(TAG, "Drawer1 connection sensor handler invoked for chattering removal.");

    int count_low = 0, count_high = 0;
    int current_value;

    do
    {
        int adc_raw;
        adc_oneshot_read(adc1_handle, DK1_SNS_CHANNEL, &adc_raw);      // Read ADC value
        float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL0) * V_REF; // Convert to voltage

        current_value = (voltage > LOGICVOL_DRAWER1) ? 1 : 0;

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

    } while (count_low < offConnSensor1 && count_high < onConnSensor1);
}

void SetupDrawer1ConnSensorPolarity(BYTE pole)
{
    drawer1Polarity = pole;
    ESP_LOGI(TAG, "Drawer1 connection sensor polarity set to %s", pole ? "HIGH" : "LOW");
}

bool IsConnectedDrawer1(void)
{
    Drawer1ConnSensorHandler();
    int adc_raw;
    adc_oneshot_read(adc1_handle, DK1_SNS_CHANNEL, &adc_raw);      // Read ADC value
    float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL0) * V_REF; // Convert to voltage
    bool currentValue = (voltage > LOGICVOL_DRAWER1) ? 1 : 0;

    if (drawer1Polarity == 0)
    {
        return !currentValue;
    }
    else
    {
        return currentValue;
    }
}
