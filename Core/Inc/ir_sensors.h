#ifndef IR_SENSORS_H
#define IR_SENSORS_H
#include "main.h"
#include <stdint.h>
#define IR_SENSOR_COUNT 8U
#define IR_ADC_MAX 4095U
extern volatile uint16_t ir_raw[IR_SENSOR_COUNT];
extern volatile uint32_t ir_frame_count;
extern volatile uint8_t ir_adc_running;
HAL_StatusTypeDef IR_Sensors_Init(void);
void IR_Sensors_Update(void);
#endif
