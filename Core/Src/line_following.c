
#include "line_following.h"
#include "line_position.h"
#include "ir_sensors.h"
#include "motor_control.h"

/* Initial tuning parameters */
#define LF_BASE_SPEED          22
#define LF_MAX_CORRECTION      18
#define LF_KP                  1.1f
#define LF_KD                  0.015f

#define LF_LOOP_MS             5U
#define LF_LOST_TIMEOUT_MS     150U
#define LF_SEARCH_SPEED        12

#define LF_INTEGRAL_UNUSED     0

volatile float line_pid_p_term = 0.0f;
volatile float line_pid_d_term = 0.0f;
volatile int16_t line_pid_output = 0;

static float previous_error = 0.0f;

static uint32_t last_update = 0U;
static uint32_t lost_since = 0U;
static uint8_t was_line_detected = 0U;

void LineFollowing_Init(void)
{
    previous_error = 0.0f;

    line_pid_p_term = 0.0f;
    line_pid_d_term = 0.0f;
    line_pid_output = 0;

    last_update = HAL_GetTick();
    lost_since = 0U;
    was_line_detected = 0U;
}

void LineFollowing_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_update;

    /* Run the controller at approximately 200 Hz. */
    if (elapsed_ms < LF_LOOP_MS) {
        return;
    }

    last_update = now;

    /* Refresh sensors and calculate the current line position. */
    IR_Sensors_Update();
    LinePosition_Update();

    /*
     * Line lost: rotate slowly to search for it.
     * Stop if the line has not been found within the timeout.
     */
    if (!line_detected) {
        if (lost_since == 0U) {
            lost_since = now;
        }

        if ((now - lost_since) >= LF_LOST_TIMEOUT_MS) {
            MotorControl_Stop();
        } else {
            /*
             * Use the last valid error to choose a search direction.
             * Assumes positive error means the line is to the right.
             * Verify motor direction on your assembled robot.
             */
            if (line_error_mm >= 0.0f) {
                MotorControl_Set(
                    LF_SEARCH_SPEED,
                    -LF_SEARCH_SPEED
                );
            } else {
                MotorControl_Set(
                    -LF_SEARCH_SPEED,
                    LF_SEARCH_SPEED
                );
            }
        }

        was_line_detected = 0U;
        return;
    }

    lost_since = 0U;

    /*
     * Reset derivative history after reacquiring the line to avoid
     * a sudden correction caused by a stale previous error.
     */
    if (!was_line_detected) {
        previous_error = line_error_mm;
        line_pid_d_term = 0.0f;
    }

    was_line_detected = 1U;

    float error = line_error_mm;

    /* Proportional term */
    line_pid_p_term = LF_KP * error;

    /* Derivative term using actual elapsed time */
    float dt_ms = (float)elapsed_ms;

    if (dt_ms > 0.0f) {
        line_pid_d_term =
            LF_KD * (error - previous_error) / dt_ms;
    } else {
        line_pid_d_term = 0.0f;
    }

    previous_error = error;

    /* PD output */
    float correction = line_pid_p_term + line_pid_d_term;

    /* Limit steering correction */
    if (correction > (float)LF_MAX_CORRECTION) {
        correction = (float)LF_MAX_CORRECTION;
    } else if (correction < -(float)LF_MAX_CORRECTION) {
        correction = -(float)LF_MAX_CORRECTION;
    }

    line_pid_output = (int16_t)correction;

    /*
     * Differential steering.
     * Positive error is assumed to mean line is to the right.
     * If the robot steers away from the line, reverse the correction
     * signs in these two motor commands.
     */
    int16_t left_speed =
        (int16_t)(LF_BASE_SPEED + line_pid_output);

    int16_t right_speed =
        (int16_t)(LF_BASE_SPEED - line_pid_output);

    MotorControl_Set(left_speed, right_speed);
}
