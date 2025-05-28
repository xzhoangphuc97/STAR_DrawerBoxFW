#ifndef COMPULSION_SW1_H
#define COMPULSION_SW1_H

#include <stdbool.h>
#include "esp_err.h"
#include "parameter.h"

// Function declarations
void InitCompulsionSw1(void);
ER_T StartCompulsionSw1Sampling(WORD onCnt, WORD offCnt);
void StopCompulsionSw1Sampling(void);
void CompulsionSw1Handler(void);
void SetupCompulsionSw1Polarity(BYTE pole);
bool IsCompulsionSw1Pushed(void);

#endif // COMPULSION_SW1_H
