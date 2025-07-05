#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"
//===========================================pin==================================================
#define MOTOR_HZ 12500
#define MOTOR_HZ_RANGE 1000

#define MOTOR_BOTTOM (MOTOR_BOTTOM_PIN)
#define DIR_BOTTOM (MOTOR_BOTTOM_DIR)
#define ENABLE_BOTTOM (MOTOR_BOTTOM_ENABLE)

//===========================================limit==================================================
#define MOMENTUM_MOTOR_PWM_MAX 9999
#define MOMENTUM_MOTOR_PWM_MIN -9999
#define MOTOR_PWM_MAX 9999
#define MOTOR_PWM_MIN -9999

extern uint32 bottom_motor_deadzone_backword;
extern uint32 bottom_motor_deadzone_forward;

void motor_init();
void set_bottom_motor_pwn(int32 pwm);
void set_bottom_motor_hertz(int32 hertz);
void stop_bottom_motor(void);

void set_momentum_motor_pwm(int32 pwmFront, int32 pwmBack);
void stop_momentum_motor(void);

#endif /* _MOTOR_H_ */