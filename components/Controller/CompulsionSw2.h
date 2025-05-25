#ifndef COMPULSION_SW2_H
#define COMPULSION_SW2_H

#include <stdbool.h>
#include "esp_err.h"
#include "defines.h"

// Function declarations
void InitCompulsionSw2(void);
ER_T StartCompulsionSw2Sampling(WORD onCnt, WORD offCnt);
void StopCompulsionSw2Sampling(void);
void CompulsionSw2Handler(void);
void SetupCompulsionSw2Polarity(BYTE pole);
bool IsCompulsionSw2Pushed(void);

#endif // COMPULSION_SW2_H
