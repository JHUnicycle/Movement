#include "motor.h"
#include "single_driver.h"

uint32 bottom_motor_deadzone_backword = 240;
uint32 bottom_motor_deadzone_forward = 240;
void motor_init()
{
    gpio_init(ENABLE_BOTTOM, GPO, 1, GPO_PUSH_PULL);
    gpio_set_level(ENABLE_BOTTOM, 1);

    pwm_init(MOTOR_BOTTOM, MOTOR_HZ, 0);
    gpio_init(DIR_BOTTOM, GPO, 1, GPO_PUSH_PULL);
}

// bottom motor
void set_bottom_motor_pwm(int32 pwm)
{
    restrictValueI(&pwm, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    if (pwm >= 0)
    {
        gpio_set_level(DIR_BOTTOM, 0);
        pwm_set_duty(MOTOR_BOTTOM, pwm);
    }
    else
    {
        gpio_set_level(DIR_BOTTOM, 1);
        pwm_set_duty(MOTOR_BOTTOM, -pwm);
    }
}

void set_bottom_motor_hertz(int32 hertz)
{
    restrictValueI(&hertz, MOTOR_HZ + MOTOR_HZ_RANGE,
                   MOTOR_HZ - MOTOR_HZ_RANGE);
    pwm_init(MOTOR_BOTTOM, hertz, 0);
}

void stop_bottom_motor(void)
{
    set_bottom_motor_pwm(0);
}

// int32 abs(int32* value) {
//     return (*value < 0) ? -*value : *value;
// }

bool restrictChange(int32 value)
{
    static int32 lastValue = 0;
    if (abs(lastValue - value) >= 2000)
    {
        lastValue = value;
        return false;
    }
    lastValue = value;
    return true;
}

// momentum motor
void set_momentum_motor_pwm(int32 pwmFront, int32 pwmBack)
{
    // WARN: 正负值可能需要调整
    restrictValueI(&pwmFront, MOMENTUM_MOTOR_PWM_MIN, MOMENTUM_MOTOR_PWM_MAX);
    restrictValueI(&pwmBack, MOMENTUM_MOTOR_PWM_MIN, MOMENTUM_MOTOR_PWM_MAX);

    // if (restrictChange(pwmFront) || restrictChange(pwmBack)) {
    //     return;
    // }

    // printf("%d, %d\n", pwmFront, pwmBack);
    small_driver_set_duty(-pwmFront, -pwmBack);

    // x轴朝前方，y轴朝右方，z轴朝下方
    // duty 为正的时候，电机转动方向为逆时针
}

void stop_momentum_motor(void)
{
    small_driver_set_duty(0, 0);
}