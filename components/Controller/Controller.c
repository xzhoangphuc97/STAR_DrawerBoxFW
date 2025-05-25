#include "Controller.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#define VM_SNS_CHANNEL ADC_CHANNEL_5
static adc_oneshot_unit_handle_t adc1_handle;

// variable
static uint8_t ABS_S1 = 0x00;
static uint8_t ABS_S2 = 0x00;
static uint8_t ABS_S3 = 0x00;
static uint8_t ABS_S4 = 0x00;
static uint8_t ABS_S1_old = 0x00;
static uint8_t ABS_S2_old = 0x00;
static uint8_t ABS_S3_old = 0x00;
static uint8_t ABS_S4_old = 0x00;
static bool changeFlagABS = false;
void InitADCChannels(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // Using ADC1
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // Configure multiple ADC channels
    InitDrawer1ADC(adc1_handle);
    InitDrawer2ADC(adc1_handle);
    InitDrawer3ADC(adc1_handle);
    InitDrawer4ADC(adc1_handle);
    ESP_LOGI(TAG, "All Connect Sensor ADC channel initialized");
}
void GetASBStatus(void)
{
    bool stateSW1 = IsCompulsionSw1Pushed();
    if (stateSW1 == true)
    {
        ABS_S1 = ABS_S1 | 0b00000100;
        ABS_S2 = ABS_S2 | 0b00100100;
    }
    else
    {
        ABS_S1 = ABS_S1 & 0b11111011;
        ABS_S2 = ABS_S2 & 0b11011011;
    }
    bool stateSW2 = IsCompulsionSw2Pushed();
    (stateSW2 == true) ? (ABS_S3 = ABS_S3 | 0b00100100) : (ABS_S3 = ABS_S3 & 0b11011011);
    bool statusConnectDrawer1 = IsConnectedDrawer1();
    bool statusConnectDrawer2 = IsConnectedDrawer2();
    bool statusConnectDrawer3 = IsConnectedDrawer3();
    bool statusConnectDrawer4 = IsConnectedDrawer4();
    (statusConnectDrawer1 == true) ? (ABS_S4 = ABS_S4 | 0b01000000) : (ABS_S4 = ABS_S4 & 0b10111111);
    (statusConnectDrawer2 == true) ? (ABS_S4 = ABS_S4 | 0b00100000) : (ABS_S4 = ABS_S4 & 0b11011111);
    (statusConnectDrawer3 == true) ? (ABS_S4 = ABS_S4 | 0b00001000) : (ABS_S4 = ABS_S4 & 0b11110111);
    (statusConnectDrawer4 == true) ? (ABS_S4 = ABS_S4 | 0b00000100) : (ABS_S4 = ABS_S4 & 0b11111011);
    if (ABS_S1 != ABS_S1_old || ABS_S2 != ABS_S2_old || ABS_S3 != ABS_S3_old || ABS_S4 != ABS_S4_old)
    {
        changeFlagABS = true;
        ABS_S1_old = ABS_S1;
        ABS_S2_old = ABS_S2;
        ABS_S3_old = ABS_S3;
        ABS_S4_old = ABS_S4;
    }
}
uint8_t GetABS_S1()
{
    return ABS_S1;
}
uint8_t GetABS_S2()
{
    return ABS_S2;
}
uint8_t GetABS_S3()
{
    return ABS_S3;
}
uint8_t GetABS_S4()
{
    return ABS_S4;
}
bool getChangeFlagStatus()
{
    return changeFlagABS;
}
void setChangeFlagStatusToFalse(void)
{
    changeFlagABS = false;
}
void InitLed(void)
{

    gpio_reset_pin(LED_BLUE);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_RED);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
}

void InitSupply(void)
{
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_CHANNEL5, // Adjust attenuation as needed
    };
    adc_oneshot_config_channel(adc1_handle, VM_SNS_CHANNEL, &config);

    ESP_LOGI(TAG, "GPIO[%d] Analog input initialized", VM_SNS_CHANNEL);
}

float VoltageMonitor(void)
{
    int adc_values[30];
    int max_value, min_value;
    uint8_t max_position = 0, min_position = 0;

    // Read 30 ADC samples every 10ms
    adc_oneshot_read(adc1_handle, VM_SNS_CHANNEL, &adc_values[0]);
    max_value = min_value = adc_values[0];

    vTaskDelay(pdMS_TO_TICKS(10));
    for (int i = 1; i < 30; i++)
    {
        adc_oneshot_read(adc1_handle, VM_SNS_CHANNEL, &adc_values[i]);

        if (adc_values[i] > max_value)
        {
            max_value = adc_values[i];
            max_position = i;
        }
        if (adc_values[i] < min_value)
        {
            min_value = adc_values[i];
            min_position = i;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Sum values excluding max & min
    long sum = 0;
    for (int i = 0; i < 30; i++)
    {
        if (i != max_position && i != min_position)
        {
            sum += adc_values[i];
        }
    }

    // Calculate average of 28 values
    float avg_adc = sum / 28.0;

    // Convert ADC value to voltage
    float voltage = avg_adc / VALUE12BITADC * V_REF * V_COVERT;
    // printf("Average ADC: %.2f, Voltage: %.2f mV \n", avg_adc, voltage);
    // ESP_LOGI(TAG, "Average ADC: %.2f, Voltage: %.2f mV", avg_adc, voltage);

    return voltage;
}
void InitPowerOut(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << VM_EN,         // Select GPIO pin 21
        .mode = GPIO_MODE_OUTPUT,              // Set as output
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up resistor
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };
    gpio_config(&io_conf);
}

void InitPushSW(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << Push_Switch),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}
void SetPowerOut(void)
{
    gpio_set_level(VM_EN, 1); // Set the GPIO level to high
}

void Configure_gpio(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode)
{
    // Reset the GPIO pin to its default state
    gpio_reset_pin(gpio_num);

    // Set the direction (mode)
    gpio_set_direction(gpio_num, mode);

    if (mode == GPIO_MODE_INPUT)
    {
        // Set the pull-up or pull-down mode if it's an input
        gpio_set_pull_mode(gpio_num, pull_mode);
    }
}
void ConfigureGpioUnsed()
{
    Configure_gpio(GPIO_UNUSE42, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
    Configure_gpio(GPIO_UNUSE45, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
    Configure_gpio(GPIO_UNUSE46, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
    Configure_gpio(GPIO_UNUSE47, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
    Configure_gpio(GPIO_UNUSE48, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
}

void BlinkLED(void)
{
    while (true)
    {
        gpio_set_level(LED_BLUE, 1);  // Turn LED ON
        gpio_set_level(LED_RED, 1);   // Turn LED ON
        gpio_set_level(LED_GREEN, 1); // Turn LED ON
        gpio_set_level(DK1_CNT, 1);   // Turn LED ON
        gpio_set_level(DK2_CNT, 1);   // Turn LED ON
        gpio_set_level(DK3_CNT, 1);   // Turn LED ON
        gpio_set_level(DK4_CNT, 1);   // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(5000));
        gpio_set_level(LED_BLUE, 0);  // Turn LED OFF
        gpio_set_level(LED_RED, 0);   // Turn LED ON
        gpio_set_level(LED_GREEN, 0); // Turn LED ON
        gpio_set_level(DK1_CNT, 0);   // Turn LED ON
        gpio_set_level(DK2_CNT, 0);   // Turn LED ON
        gpio_set_level(DK3_CNT, 0);   // Turn LED ON
        gpio_set_level(DK4_CNT, 0);   // Turn LED ON
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void TestingControl(void)
{
    while (1)
    {
        if (IsCompulsionSw1Pushed() == 1 && IsCompulsionSw2Pushed() == 0)
        {
            gpio_set_level(LED_GREEN, 1);
        }
        else
        {
            gpio_set_level(LED_GREEN, 0);
        }
        if (IsCompulsionSw2Pushed() == 1 && IsCompulsionSw1Pushed() == 0)
        {
            gpio_set_level(LED_RED, 1);
        }
        else
        {
            gpio_set_level(LED_RED, 0);
        }
        if (IsCompulsionSw2Pushed() == 1 && IsCompulsionSw1Pushed() == 1)
        {
            gpio_set_level(LED_BLUE, 1);
        }
        else
        {
            gpio_set_level(LED_BLUE, 0);
        }

        if (IsConnectedDrawer1() == 1)
        {
            ESP_LOGI(TAG, "Drive 1!");
            DriveDrawer1(1000, 1000);
        }
        if (IsConnectedDrawer2() == 1)
        {
            ESP_LOGI(TAG, "Drive 2!");
            DriveDrawer2(1000, 1000);
        }
        if (IsConnectedDrawer3() == 1)
        {
            ESP_LOGI(TAG, "Drive 3!");
            DriveDrawer3(1000, 1000);
        }
        if (IsConnectedDrawer4() == 1)
        {
            ESP_LOGI(TAG, "Drive 4!");
            DriveDrawer4(1000, 1000);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
static int n1 = 20;
static int n2 = 20;

// Setting the cash drawer 1 drive pulse width
static bool Drawer1Run = false;
static bool Drawer2Run = false;
static bool Drawer3Run = false;
static bool Drawer4Run = false;
void SettingPulseWidthDrawer1(int value1, int value2)
{
    ESP_LOGI(TAG, "Set onTime, offTime for Drawer1 complete!");
    n1 = value1;
    n2 = value2;
}

// Cash drawer 1 drive command
void DriveCommandDrawer1()
{
    uint8_t count = 0;
    if (!IsConnectedDrawer1())
    {
        ESP_LOGE(TAG, "Can not run drawer 1 because of disconected!");
    }
    else
    {
        while (VoltageMonitor() < VOLTAGE_THRESHOLD && count < 100)
            count++;
        if (count < 100)
        {
            Drawer1Run = true;
            DriveDrawer1(n1 * 10, n2 * 10);
            Drawer1Run = false;
        }
        else
        {
            ESP_LOGE(TAG, "Can not run drawer 1 because of VN-SNS!");
        }
    }
}

void DriveCommandDrawer2()
{
    uint8_t count = 0;
    if (!IsConnectedDrawer2())
    {
        ESP_LOGE(TAG, "Can not run drawer 2 because of disconected!");
    }
    else
    {
        while (VoltageMonitor() < VOLTAGE_THRESHOLD && count < 100)
            count++;
        if (count < 100)
        {
            Drawer2Run = true;
            DriveDrawer2(200, 200);
            Drawer2Run = false;
        }
        else
        {
            ESP_LOGE(TAG, "Can not run drawer 2 because of VN-SNS!");
        }
    }
}

void DriveCommandDrawer3()
{
    uint8_t count = 0;
    if (!IsConnectedDrawer3())
    {
        ESP_LOGE(TAG, "Can not run drawer 3 because of disconected!");
    }
    else
    {
        while (VoltageMonitor() < VOLTAGE_THRESHOLD && count < 100)
            count++;
        if (count < 100)
        {
            Drawer3Run = true;
            DriveDrawer3(200, 200);
            Drawer3Run = false;
        }
        else
        {
            ESP_LOGE(TAG, "Can not run drawer 3 because of VN-SNS!");
        }
    }
}

void DriveCommandDrawer4()
{
    uint8_t count = 0;
    if (!IsConnectedDrawer4())
    {
        ESP_LOGE(TAG, "Can not run drawer 2 because of disconected!");
    }
    else
    {
        while (VoltageMonitor() < VOLTAGE_THRESHOLD && count < 100)
            count++;
        if (count < 100)
        {
            Drawer4Run = true;
            DriveDrawer4(200, 200);
            Drawer4Run = false;
        }
        else
        {
            ESP_LOGE(TAG, "Can not run drawer 4 because of VN-SNS!");
        }
    }
}

void ExecuteCommand(char *input)
{
    // get from common buffer
    char *tokens[10]; // Array to store extracted tokens
    int count = 0;

    char *token = strtok(input, " "); // Split by spaces
    while (token != NULL && count < 10)
    {
        tokens[count++] = token;
        token = strtok(NULL, " ");
    }

    if (count == 4 && strcmp(tokens[0], ESC) == 0 && strcmp(tokens[1], BEL) == 0) // Do Command for ESC BEL n1 n2
    {
        unsigned int hexValue2 = strtol(tokens[2], NULL, 16);
        unsigned int hexValue3 = strtol(tokens[3], NULL, 16);
        SettingPulseWidthDrawer1(hexValue2, hexValue3);
    }
    else if (count == 1 && strcmp(tokens[0], BEL) == 0) // Do Command for BEL
    {
        DriveCommandDrawer1();
    }
    else if (count == 1 && strcmp(tokens[0], FS) == 0) // Do Command for FS
    {
        DriveCommandDrawer1();
    }
    else if (count == 1 && strcmp(tokens[0], SUB) == 0) // Do Command for SUB
    {
        DriveCommandDrawer2();
    }
    else if (count == 1 && strcmp(tokens[0], EM) == 0) // Do Command for EM
    {
        DriveCommandDrawer2();
    }
    else if (count == 1 && strcmp(tokens[0], DC3) == 0) // Do Command for DC3
    {
        DriveCommandDrawer3();
    }
    else if (count == 1 && strcmp(tokens[0], DC4) == 0) // Do Command for DC4
    {
        DriveCommandDrawer4();
    }
}
void ForceStopDrawer(float CurrentVoltage)
{
    if (CurrentVoltage < VOLTAGE_THRESHOLD && (Drawer1Run == true || Drawer2Run == true || Drawer3Run == true || Drawer4Run == true))
    {
        ESP_LOGE(TAG, "Force Stop Drawer Run! Current voltage: %.2f", CurrentVoltage);
        if (Drawer1Run == true)
            OffDrawer1();
        if (Drawer2Run == true)
            OffDrawer2();
        if (Drawer3Run == true)
            OffDrawer3();
        if (Drawer4Run == true)
            OffDrawer4();
        Drawer1Run = false;
        Drawer2Run = false;
        Drawer3Run = false;
        Drawer4Run = false;
    }
}
bool getASBFlag(void)
{
    return RealTimeASB;
}
