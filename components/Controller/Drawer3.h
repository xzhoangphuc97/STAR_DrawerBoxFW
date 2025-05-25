#ifndef DRAWER3_H
#define DRAWER3_H

#include "driver/gpio.h"
#include "defines.h"

void InitDrawer3(void);                               // Declaration of the function
ER_T DriveDrawer3(uint16_t onTime, uint16_t offTime); // Drive Drawer 3
void OffDrawer3(void);

#endif // DRAWER3_H
