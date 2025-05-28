#ifndef DRAWER3_CONN_SENSOR_H
#define DRAWER3_CONN_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"
#include "parameter.h"

// Function Declarations
void InitDrawer3ADC(adc_oneshot_unit_handle_t adc_handle);
void InitDrawer3ConnSensor(void);
ER_T StartDrawer3ConnSensorSampling(WORD onCnt, WORD offCnt);
void StopDrawer3ConnSensorSampling(void);
void Drawer3ConnSensorHandler(void);
void SetupDrawer3ConnSensorPolarity(BYTE pole);
bool IsConnectedDrawer3(void);

#endif // DRAWER3_CONN_SENSOR_H
