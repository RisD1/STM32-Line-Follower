#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "main.h"
#include <stdint.h>
typedef enum { MOTOR_DIR_BACKWARD = -1, MOTOR_DIR_STOP = 0, MOTOR_DIR_FORWARD = 1 } MotorDirection;
void MotorControl_Init(void);
void MotorControl_Stop(void);
void MotorControl_Set(int16_t left_percent, int16_t right_percent);
void MotorControl_Forward(uint16_t percent);
void MotorControl_Backward(uint16_t percent);
/* Timed turns are calibration placeholders, not guaranteed 90-degree turns until tuned. */
void MotorControl_TurnLeft90_Start(void);
void MotorControl_TurnRight90_Start(void);
void MotorControl_TurnUpdate(void);
extern volatile uint8_t motor_turn_active;
#endif
