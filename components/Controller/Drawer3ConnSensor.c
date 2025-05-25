#include "Drawer3ConnSensor.h"
#include "driver/gpio.h"
#include "esp_log.h"


static const char *TAG = "Drawer3ConnSensor";

#define DK3_SNS_CHANNEL ADC_CHANNEL_3
static adc_oneshot_unit_handle_t adc1_handle;


static BYTE drawer3Polarity = 1; // Default polarity: HIGH = Connected
static WORD onConnSensor3 = 0;
static WORD offConnSensor3 = 0;
static bool isConnSensor3Initialized = false; // Initialization flag
void InitDrawer3ADC(adc_oneshot_unit_handle_t adc_handle)
{
    adc1_handle = adc_handle;
}
void InitDrawer3ConnSensor(void) {


    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_CHANNEL3,  // Adjust attenuation as needed
    };
    adc_oneshot_config_channel(adc1_handle, DK3_SNS_CHANNEL, &config);

    isConnSensor3Initialized = true;
    ESP_LOGI(TAG, "GPIO[%d] Analog input initialized", DK3_SNS_CHANNEL);
}



ER_T StartDrawer3ConnSensorSampling(WORD onCnt, WORD offCnt)
{
    if (onCnt < 1 || onCnt > 65535 || offCnt < 1 || offCnt > 65535)
    {
        ESP_LOGE(TAG, "Invalid arguments: onCnt=%d, offCnt=%d", onCnt, offCnt);
        return ER_PAR;
    }
    onConnSensor3 = onCnt;
    offConnSensor3 = offCnt;
    return ER_OK;
}

void StopDrawer3ConnSensorSampling(void)
{
    ESP_LOGI(TAG, "Drawer3 connection sensor sampling stopped.");
}

void Drawer3ConnSensorHandler(void) {
    ESP_LOGD(TAG, "Drawer3 connection sensor handler invoked for chattering removal.");

    int count_low = 0, count_high = 0;
    int current_value;

    do {
        int adc_raw;
        adc_oneshot_read(adc1_handle, DK3_SNS_CHANNEL, &adc_raw);  // Read ADC value
        float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL3) * V_REF; // Convert to voltage
        current_value = (voltage > LOGICVOL_DRAWER3) ? 1 : 0;

        if (current_value == 0) { // LOW state
            count_low++;
            count_high = 0; // Reset high counter
        } else { // HIGH state
            count_high++;
            count_low = 0; // Reset low counter
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for stability (adjust as needed)

    } while (count_low < onConnSensor3 && count_high < offConnSensor3);
}



void SetupDrawer3ConnSensorPolarity(BYTE pole)
{
    drawer3Polarity = pole;
    ESP_LOGI(TAG, "Drawer3 connection sensor polarity set to %s", pole ? "HIGH" : "LOW");
}

bool IsConnectedDrawer3(void)
{
    Drawer3ConnSensorHandler();
int adc_raw;
        adc_oneshot_read(adc1_handle, DK3_SNS_CHANNEL, &adc_raw);  // Read ADC value
        float voltage = ((float)adc_raw / ADC_VALUE_CHANNEL3) * V_REF; // Convert to voltage
    // ESP_LOGI(TAG, "Drawer3 connection sensor voltage: %f", voltage);
    bool currentValue = (voltage > LOGICVOL_DRAWER1) ? 1 : 0;

    if (drawer3Polarity == 0)
    {
        return !currentValue;
    }
    else
    {
        return currentValue;
    }
}
