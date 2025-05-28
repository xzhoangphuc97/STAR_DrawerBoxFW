#ifndef DRAWER1_CONN_SENSOR_H
#define DRAWER1_CONN_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "parameter.h"


// Function Declarations
void InitDrawer1ADC(adc_oneshot_unit_handle_t adc_handle);
void InitDrawer1ConnSensor(void);
ER_T StartDrawer1ConnSensorSampling(WORD onCnt, WORD offCnt);
void StopDrawer1ConnSensorSampling(void);
void Drawer1ConnSensorHandler(void);
void SetupDrawer1ConnSensorPolarity(BYTE pole);
bool IsConnectedDrawer1(void);

#endif // DRAWER1_CONN_SENSOR_H
