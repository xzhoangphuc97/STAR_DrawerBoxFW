#ifndef DRAWER2_CONN_SENSOR_H
#define DRAWER2_CONN_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "defines.h"

// Function Declarations
void InitDrawer2ADC(adc_oneshot_unit_handle_t adc_handle);

void InitDrawer2ConnSensor(void);
ER_T StartDrawer2ConnSensorSampling(WORD onCnt, WORD offCnt);
void StopDrawer2ConnSensorSampling(void);
void Drawer2ConnSensorHandler(void);
void SetupDrawer2ConnSensorPolarity(BYTE pole);
bool IsConnectedDrawer2(void);

#endif // DRAWER2_CONN_SENSOR_H
