#ifndef LINE_POSITION_H
#define LINE_POSITION_H
#include "main.h"
#include <stdint.h>
#include "ir_sensors.h"
/* Sensor centres are nominally 5.08 mm apart; S1 is leftmost. */
#define LINE_SENSOR_SPACING_MM 5.08f
#define LINE_ERROR_LIMIT_MM 17.78f
extern volatile float line_error_mm;
extern volatile float line_error_deviation_mm;
extern volatile uint8_t line_detected;
extern volatile uint16_t line_weight[IR_SENSOR_COUNT];
/* Set these after observing ADC values on black and white for each sensor. */
extern uint16_t sensor_black[IR_SENSOR_COUNT];
extern uint16_t sensor_white[IR_SENSOR_COUNT];
void LinePosition_Init(void);
void LinePosition_Update(void);
#endif
