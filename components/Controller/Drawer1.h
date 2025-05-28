#ifndef DRAWER1_H
#define DRAWER1_H

#include "driver/gpio.h"
#include "parameter.h"

void InitDrawer1(void);                               // Declaration of the function
ER_T DriveDrawer1(uint16_t onTime, uint16_t offTime); // Drive Drawer 1
void OffDrawer1(void);

#endif // DRAWER1_H
