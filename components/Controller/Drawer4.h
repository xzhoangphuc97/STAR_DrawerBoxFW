#ifndef DRAWER4_H
#define DRAWER4_H

#include "driver/gpio.h"
#include "defines.h"

void InitDrawer4(void);                               // Declaration of the function
ER_T DriveDrawer4(uint16_t onTime, uint16_t offTime); // Drive Drawer 4
void OffDrawer4(void);

#endif // DRAWER4_H
