
#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include "main.h"
#include "ir_sensors.h"

#ifndef LINE_SENSOR_SPACING_MM
#define LINE_SENSOR_SPACING_MM 5.08f
#endif

extern volatile float line_error_mm;
extern volatile float line_error_deviation_mm;
extern volatile uint8_t line_detected;
extern volatile uint16_t line_weight[IR_SENSOR_COUNT];

extern uint16_t sensor_black[IR_SENSOR_COUNT];
extern uint16_t sensor_white[IR_SENSOR_COUNT];

void LinePosition_Init(void);
void LinePosition_Update(void);

#endif /* LINE_POSITION_H */
