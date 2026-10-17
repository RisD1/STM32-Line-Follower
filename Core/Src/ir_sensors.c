#include "ir_sensors.h"
#include "adc.h"
static uint16_t adc_dma[IR_SENSOR_COUNT];
volatile uint16_t ir_raw[IR_SENSOR_COUNT] = {0};
volatile uint32_t ir_frame_count = 0;
volatile uint8_t ir_adc_running = 0;
HAL_StatusTypeDef IR_Sensors_Init(void) {
    HAL_StatusTypeDef s = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, IR_SENSOR_COUNT);
    ir_adc_running = (s == HAL_OK) ? 1U : 0U;
    return s;
}
void IR_Sensors_Update(void) {
    uint32_t i;
    for (i = 0; i < IR_SENSOR_COUNT; ++i) ir_raw[i] = adc_dma[i];
    ++ir_frame_count;
}
