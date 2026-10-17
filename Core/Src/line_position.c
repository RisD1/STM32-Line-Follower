#include "line_position.h"
#include <math.h>
volatile float line_error_mm = 0.0f;
volatile float line_error_deviation_mm = 0.0f;
volatile uint8_t line_detected = 0;
volatile uint16_t line_weight[IR_SENSOR_COUNT] = {0};
/* Safe uncalibrated defaults; replace with measured black/white readings. */
uint16_t sensor_black[IR_SENSOR_COUNT] = {500,500,500,500,500,500,500,500};
uint16_t sensor_white[IR_SENSOR_COUNT] = {3000,3000,3000,3000,3000,3000,3000,3000};
void LinePosition_Init(void) { line_error_mm = 0.0f; line_error_deviation_mm = 0.0f; line_detected = 0U; }
void LinePosition_Update(void) {
    uint32_t i, total = 0U, cumulative = 0U;
    int32_t span;
    float pos[IR_SENSOR_COUNT];
    /* Normalize so white-line response = 0..1000 independent of ADC polarity. */
    for (i=0; i<IR_SENSOR_COUNT; ++i) {
        uint16_t raw = ir_raw[i], b = sensor_black[i], w = sensor_white[i];
        span = (int32_t)w - (int32_t)b;
        if (span == 0) line_weight[i] = 0U;
        else {
            int32_t v = (((int32_t)raw - (int32_t)b) * 1000) / span;
            if (v < 0) v = 0; if (v > 1000) v = 1000;
            line_weight[i] = (uint16_t)v;
        }
        total += line_weight[i];
        pos[i] = ((float)i - 3.5f) * LINE_SENSOR_SPACING_MM;
    }
    if (total < 180U) { line_detected = 0U; line_error_deviation_mm = 0.0f; return; }
    /* Weighted median: location where cumulative line weight reaches half. */
    for (i=0; i<IR_SENSOR_COUNT; ++i) {
        cumulative += line_weight[i];
        if (cumulative * 2U >= total) { line_error_mm = pos[i]; break; }
    }
    line_detected = 1U;
    /* Deviation allowance: distance of estimate from centre, beyond deadband. */
    line_error_deviation_mm = fabsf(line_error_mm);
}
