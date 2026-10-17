#ifndef SWITCH_INPUT_H
#define SWITCH_INPUT_H
#include "main.h"
#include <stdint.h>
/* Logical bit is 1 when switch is ON. Electrical input is active-low. */
extern volatile uint8_t switch_bits[4];
extern volatile uint8_t switch_value;
void SwitchInput_Init(void);
void SwitchInput_Update(void);
#endif
