
#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "main.h"

extern volatile float line_pid_p_term;
extern volatile float line_pid_d_term;
extern volatile int16_t line_pid_output;

void LineFollowing_Init(void);
void LineFollowing_Update(void);

#endif /* LINE_FOLLOWING_H */
