#ifndef DRAWER2_H
#define DRAWER2_H

#include "driver/gpio.h"
#include "parameter.h"

void InitDrawer2(void);                               // Declaration of the function
ER_T DriveDrawer2(uint16_t onTime, uint16_t offTime); // Drive Drawer 2
void OffDrawer2(void);

#endif // DRAWER2_H
