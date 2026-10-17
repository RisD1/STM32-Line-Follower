#include "line_following.h"
#include "line_position.h"
#include "ir_sensors.h"
#include "motor_control.h"
#define LF_BASE_SPEED 22
#define LF_MAX_CORRECTION 18
#define LF_KP 1.1f /* initial, uncalibrated */
#define LF_KI 0.0f /* start with PD; add I only if tests justify it */
#define LF_KD 0.015f /* initial, uncalibrated; depends on loop period */
#define LF_LOOP_MS 5U
#define LF_LOST_TIMEOUT_MS 150U
volatile float line_pid_p_term=0.0f, line_pid_d_term=0.0f;
volatile int16_t line_pid_output=0;
static float integral=0.0f, previous_error=0.0f;
static uint32_t last_update=0U, lost_since=0U;
void LineFollowing_Init(void) { integral=0.0f; previous_error=0.0f; last_update=HAL_GetTick(); lost_since=0U; }
void LineFollowing_Update(void) {
    uint32_t now=HAL_GetTick(); float e, derivative, correction; int16_t c;
    if ((uint32_t)(now-last_update)<LF_LOOP_MS) return;
    last_update=now;
    IR_Sensors_Update(); LinePosition_Update();
    if (!line_detected) {
        if (lost_since==0U) lost_since=now;
        if ((uint32_t)(now-lost_since)>LF_LOST_TIMEOUT_MS) MotorControl_Stop();
        else MotorControl_Set(12,-12); /* brief slow search; stop if not reacquired */
        return;
    }
    lost_since=0U; e=line_error_mm;
    derivative=(e-previous_error)/(float)LF_LOOP_MS; previous_error=e;
    integral += e*(float)LF_LOOP_MS;
    if (integral>1000.0f) integral=1000.0f; if(integral< -1000.0f) integral=-1000.0f;
    line_pid_p_term=LF_KP*e; line_pid_d_term=LF_KD*derivative;
    correction=line_pid_p_term + line_pid_d_term + LF_KI*integral;
    if(correction>LF_MAX_CORRECTION) correction=LF_MAX_CORRECTION;
    if(correction< -LF_MAX_CORRECTION) correction=-LF_MAX_CORRECTION;
    c=(int16_t)correction; line_pid_output=c;
    /* Positive error means line is to the robot's right: turn right. */
    MotorControl_Set(LF_BASE_SPEED+c, LF_BASE_SPEED-c);
}
