
#include "line_position.h"
#include <stdint.h>
#include <math.h>

volatile float line_error_mm = 0.0f;
volatile float line_error_deviation_mm = 0.0f;
volatile uint8_t line_detected = 0U;

volatile uint16_t line_weight[IR_SENSOR_COUNT] = {0U};

/*
 * Initial calibration estimates.
 * Replace each entry with the measured value for that sensor.
 * Assumption: black line on white background.
 */
uint16_t sensor_black[IR_SENSOR_COUNT] = {
    150U, 150U, 150U, 150U,
    150U, 150U, 150U, 150U
};

uint16_t sensor_white[IR_SENSOR_COUNT] = {
    600U, 600U, 600U, 600U,
    600U, 600U, 600U, 600U
};

/* Minimum combined weight required to consider the line detected. */
#define LINE_MIN_TOTAL_WEIGHT 180U

void LinePosition_Init(void)
{
    line_error_mm = 0.0f;
    line_error_deviation_mm = 0.0f;
    line_detected = 0U;

    for (uint8_t i = 0U; i < IR_SENSOR_COUNT; i++) {
        line_weight[i] = 0U;
    }
}

void LinePosition_Update(void)
{
    uint32_t total_weight = 0U;

    /*
     * Convert each ADC reading into a line-detection weight.
     *
     * Black -> approximately 1000
     * White -> approximately 0
     */
    for (uint8_t i = 0U; i < IR_SENSOR_COUNT; i++) {
        int32_t raw = (int32_t)ir_raw[i];
        int32_t black = (int32_t)sensor_black[i];
        int32_t white = (int32_t)sensor_white[i];
        int32_t span = white - black;

        if (span <= 0) {
            line_weight[i] = 0U;
            continue;
        }

        int32_t value = ((white - raw) * 1000) / span;

        if (value < 0) {
            value = 0;
        } else if (value > 1000) {
            value = 1000;
        }

        line_weight[i] = (uint16_t)value;
        total_weight += line_weight[i];
    }

    /*
     * No convincing line detection.
     * Preserve the last valid line_error_mm so the line-following
     * controller can use the previous direction when searching.
     */
    if (total_weight < LINE_MIN_TOTAL_WEIGHT) {
        line_detected = 0U;
        line_error_deviation_mm = 0.0f;
        return;
    }

    /*
     * Weighted median: find the sensor where cumulative weight
     * reaches half the total.
     *
     * Sensor order must run from left to right.
     */
    uint32_t cumulative_weight = 0U;
    uint32_t median_threshold = (total_weight + 1U) / 2U;

    for (uint8_t i = 0U; i < IR_SENSOR_COUNT; i++) {
        cumulative_weight += line_weight[i];

        if (cumulative_weight >= median_threshold) {
            line_error_mm =
                ((float)i -
                 ((float)IR_SENSOR_COUNT - 1.0f) / 2.0f)
                * LINE_SENSOR_SPACING_MM;

            break;
        }
    }

    line_error_deviation_mm = fabsf(line_error_mm);
    line_detected = 1U;
}
