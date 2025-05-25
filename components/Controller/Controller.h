#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "defines.h"
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "Drawer1.h"
#include "Drawer2.h"
#include "Drawer3.h"
#include "Drawer4.h"
#include "CompulsionSw1.h"
#include "CompulsionSw2.h"
#include "Drawer1ConnSensor.h"
#include "Drawer2ConnSensor.h"
#include "Drawer3ConnSensor.h"
#include "Drawer4ConnSensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "CONTROL";
void InitADCChannels(void);
void ForceStopDrawer(float CurrentVotage);
void GetASBStatus(void);
float VoltageMonitor(void);
void InitLed(void);
void InitSupply(void);
void InitPowerOut(void);
void InitPushSW(void);
void ConfigureGpioUnsed();
void SetPowerOut(void);
void Configure_gpio(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode);
void BlinkLED(void);
void TestingControl(void);
void ExecuteCommand(char *input);
uint8_t GetABS_S1();
uint8_t GetABS_S2();
uint8_t GetABS_S3();
uint8_t GetABS_S4();
bool getChangeFlagStatus();
void setChangeFlagStatusToFalse(void);
bool getASBFlag(void);
#endif
