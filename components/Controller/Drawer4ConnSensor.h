#ifndef DRAWER4_CONN_SENSOR_H
#define DRAWER4_CONN_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "defines.h"

// Function Declarations
void InitDrawer4ADC(adc_oneshot_unit_handle_t adc_handle);
void InitDrawer4ConnSensor(void);
ER_T StartDrawer4ConnSensorSampling(WORD onCnt, WORD offCnt);
void StopDrawer4ConnSensorSampling(void);
void Drawer4ConnSensorHandler(void);
void SetupDrawer4ConnSensorPolarity(BYTE pole);
bool IsConnectedDrawer4(void);

#endif // DRAWER4_CONN_SENSOR_H
