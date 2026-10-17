#include "motor_control.h"
#include "tim.h"
#include "gpio.h"
#define PWM_TOP 4999U
/* Calibrate these on the actual chassis; time turns vary with battery and floor. */
#define TURN_PWM_PERCENT 22U
#define TURN_LEFT_MS 350U
#define TURN_RIGHT_MS 350U
volatile uint8_t motor_turn_active = 0U;
static uint32_t turn_deadline = 0U;
static void motor_dir(GPIO_PinState ain1, GPIO_PinState ain2, GPIO_PinState bin1, GPIO_PinState bin2) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, ain1); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, ain2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, bin1); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, bin2);
}
static uint16_t pwm(uint16_t pct) { if (pct > 100U) pct=100U; return (uint16_t)(((uint32_t)pct * PWM_TOP)/100U); }
void MotorControl_Init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    MotorControl_Stop();
}
void MotorControl_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    motor_dir(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); motor_turn_active=0U;
}
void MotorControl_Set(int16_t left_percent, int16_t right_percent) {
    if (left_percent > 100) left_percent=100; if (left_percent < -100) left_percent=-100;
    if (right_percent > 100) right_percent=100; if (right_percent < -100) right_percent=-100;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, left_percent >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, left_percent >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, right_percent >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, right_percent >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm((uint16_t)(left_percent < 0 ? -left_percent : left_percent)));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm((uint16_t)(right_percent < 0 ? -right_percent : right_percent)));
}
void MotorControl_Forward(uint16_t percent) { if(percent>100U)percent=100U; MotorControl_Set((int16_t)percent,(int16_t)percent); }
void MotorControl_Backward(uint16_t percent) { if(percent>100U)percent=100U; MotorControl_Set(-(int16_t)percent,-(int16_t)percent); }
void MotorControl_TurnLeft90_Start(void) { if (motor_turn_active) return; MotorControl_Set(-(int16_t)TURN_PWM_PERCENT,(int16_t)TURN_PWM_PERCENT); turn_deadline=HAL_GetTick()+TURN_LEFT_MS; motor_turn_active=1U; }
void MotorControl_TurnRight90_Start(void) { if (motor_turn_active) return; MotorControl_Set((int16_t)TURN_PWM_PERCENT,-(int16_t)TURN_PWM_PERCENT); turn_deadline=HAL_GetTick()+TURN_RIGHT_MS; motor_turn_active=1U; }
void MotorControl_TurnUpdate(void) { if(motor_turn_active && (int32_t)(HAL_GetTick()-turn_deadline)>=0) MotorControl_Stop(); }
