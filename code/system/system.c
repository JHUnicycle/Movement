#include "system.h"
#include "attitude.h"
#include "control.h"
#include "lcd.h"
#include "menu.h"
#include "menu_input.h"
#include "single_driver.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "key.h"
#include "diode.h"
#include "switch.h"
#include "receiver.h"

RunState_t runState;
uint32 g_system_attitude_cnt = 0;
uint32 g_pit_ccu61_ch1_cnt = 0;
uint32 g_pit_ccu60_ch0_cnt = 0;
uint32 g_main_1_cnt = 0;

void system_init()
{
    // ===================== DEVICE ======================== //
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口
    motor_init();
    small_driver_uart_init();
    encoder_init();
    lcd_init();
    mt9v03x_init();
    // mt9v03x2_init();
    imu_init();
    // receiver_init();
    // wireless_init();
    diode_init();
    switch_init();
    key_init_rewrite(KEY_NUM);

    // ===================== PARAMS ======================== //
    attitude_init(ATTITUDE_MAHONY);
    menu_manual_param_init();
    velocity_init(&g_vel_motor);
    control_manual_param_init();

    // ===================== PIT ======================== //
    // key
    pit_ms_init(CCU60_CH1, PIT_KEY_T);
    pit_enable(CCU60_CH1);
    // vel
    pit_ms_init(CCU60_CH0, PIT_VELOCITY_T);
    pit_enable(CCU60_CH0);
    // control
    pit_ms_init(CCU61_CH1, PIT_CONTROL_T);
    pit_enable(CCU61_CH1);
    // attitude
    pit_ms_init(CCU61_CH0, PIT_ATTITUDE_T);
    pit_enable(CCU61_CH0);
    // ===================== MENU ======================== //
    Read_EEPROM();
    // menu
    MainMenu_Set();
    menu_get_params(&g_euler_angle_bias, &g_control_time,
                    &g_control_turn_manual_params, &g_control_motion_params);

    // control init
    control_init(&g_control_motion_params);
}

void system_attitude_timer(
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Target *control_target,
    struct Velocity_Motor *vel_motor,
    struct EulerAngle *euler_angle,
    struct IMU_DATA *imu_data)
{
    g_system_attitude_cnt++;
    static uint8 cnt = 0;
    cnt++;
    // if (cnt >= 2)
    // {
    //     cnt = 0;
    //     g_attitude_cal_flag = 1;
    attitude_cal_amend(control_turn_params, control_target, vel_motor,
                       euler_angle, imu_data);
    // }
    // else
    // {
    //     g_attitude_cal_flag = 0;
    // }
}

void bottom_control_timer(struct Control_Time *control_time,
                          struct Control_Flag *control_flag,
                          struct Control_Target *control_target,
                          struct Velocity_Motor *vel_motor,
                          struct EulerAngle *euler_angle_bias,
                          struct Control_Motion_Manual_Parmas *control_motion_params,
                          struct Control_Turn_Manual_Params *control_turn_params)
{
    uint32 bottom_angle_vel_time = control_time->bottom[0];
    uint32 bottom_angle_time = control_time->bottom[1];
    uint32 bottom_vel_time = control_time->bottom[2];

    control_flag->bottom_angle_cnt++;
    control_flag->bottom_angle_vel_cnt++;
    control_flag->bottom_vel_cnt++;
    if (control_flag->bottom_angle_cnt >= bottom_angle_time)
    { // 50ms
        control_flag->bottom_angle = 1;
        control_flag->bottom_angle_cnt = 0;
    }
    else
    {
        control_flag->bottom_angle = 0;
    }
    if (control_flag->bottom_angle_vel_cnt >=
        bottom_angle_vel_time)
    { // 10ms
        control_flag->bottom_angle_vel = 1;
        control_flag->bottom_angle_vel_cnt = 0;
    }
    else
    {
        control_flag->bottom_angle_vel = 0;
    }
    if (control_flag->bottom_vel_cnt >= bottom_vel_time)
    { // 2ms
        control_flag->bottom_vel = 1;
        control_flag->bottom_vel_cnt = 0;
    }
    else
    {
        control_flag->bottom_vel = 0;
    }
    control_bottom_balance(control_target,
                           control_flag,
                           vel_motor,
                           euler_angle_bias,
                           control_motion_params,
                           control_turn_params);
}

void side_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Control_Turn_Manual_Params *control_turn_params,
                        struct Velocity_Motor *vel_motor,
                        struct EulerAngle *euler_angle_bias,
                        struct Control_Motion_Manual_Parmas *control_motion_params)
{
    uint32 side_angle_time = control_time->side[0];
    uint32 side_angle_vel_time = control_time->side[1];
    uint32 side_vel_time = control_time->side[2];

    control_flag->side_vel_cnt++;
    control_flag->side_angle_cnt++;
    control_flag->side_angle_vel_cnt++;

    if (control_flag->side_vel_cnt >= side_vel_time)
    {
        control_flag->side_vel = 1;
        control_flag->side_vel_cnt = 0;
    }
    else
    {
        control_flag->side_vel = 0;
    }
    if (control_flag->side_angle_cnt >= side_angle_time)
    {
        control_flag->side_angle = 1;
        control_flag->side_angle_cnt = 0;
    }
    else
    {
        control_flag->side_angle = 0;
    }
    if (control_flag->side_angle_vel_cnt >= side_angle_vel_time)
    {
        control_flag->side_angle_vel = 1;
        control_flag->side_angle_vel_cnt = 0;
    }
    else
    {
        control_flag->side_angle_vel = 0;
    }
    control_side_balance(control_target, control_flag, control_turn_params,
                         vel_motor, euler_angle_bias, control_motion_params);
}

void turn_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Control_Turn_Manual_Params *control_turn_params,
                        struct Control_Motion_Manual_Parmas *control_motion_params,
                        struct Velocity_Motor *vel_motor)
{
    uint32 turn_angle_vel_time = control_time->turn[0];
    uint32 turn_err_time = control_time->turn[1];
    uint32 turn_vel_time = control_time->turn[2];
    uint32 turn_angle_time = control_time->turn[3];

    control_flag->turn_angle_cnt++;
    control_flag->turn_angle_vel_cnt++;
    control_flag->turn_vel_cnt++;
    control_flag->turn_err_cnt++;

    // TAV
    if (control_flag->turn_angle_vel_cnt >= turn_angle_vel_time)
    {
        control_flag->turn_angle_vel = 1;
        control_flag->turn_angle_vel_cnt = 0;
    }
    else
    {
        control_flag->turn_angle_vel = 0;
    }

    // TE
    if (control_flag->turn_err_cnt >= turn_err_time)
    {
        control_flag->turn_err = 1;
        control_flag->turn_err_cnt = 0;
    }
    else
    {
        control_flag->turn_err = 0;
    }

    // if (control_flag->turn_vel_cnt >= turn_vel_time)
    // {
    //     control_flag->turn_vel = 1;
    //     control_flag->turn_vel_cnt = 0;
    // }
    // else
    // {
    //     control_flag->turn_vel = 0;
    // }

    // if (control_flag->turn_angle_cnt >= turn_angle_time)
    // {
    //     control_flag->turn_angle = 1;
    //     control_flag->turn_angle_cnt = 0;
    // }
    // else
    // {
    //     control_flag->turn_angle = 0;
    // }

    // 控制转向
    control_turn(control_target, control_flag, control_turn_params, control_motion_params, vel_motor);
}

void system_set_runstate(RunState_t state)
{
    // 根据不同的车辆状态执行不同的控制操作
    switch (state)
    {
    case CAR_STOP:
        runState = CAR_STOP;

        pit_disable(CCU61_CH1); // 失能控制中断
        pit_enable(CCU60_CH1);  // 使能按键中断

        stop_bottom_motor();
        stop_momentum_motor();
        lcd_clear();
        zf_log(0, "shutdown");
        break;
    case CAR_RUNNING:
        runState = CAR_RUNNING;

        // pit_enable(CCU61_CH1); // 使能控制中断
        pit_disable(CCU60_CH1); // 失能按键中断
        break;
    }
}

void system_control_timer()
{
    if (g_control_bottom_flag != 0)
    {
        bottom_control_timer(&g_control_time,
                             &g_control_flag,
                             &g_control_target,
                             &g_vel_motor,
                             &g_euler_angle_bias,
                             &g_control_motion_params,
                             &g_control_turn_manual_params);
    }

    // if (g_control_turn_flag != 0)
    // {
    turn_control_timer(&g_control_time,
                       &g_control_flag,
                       &g_control_target,
                       &g_control_turn_manual_params,
                       &g_control_motion_params,
                       &g_vel_motor);
    // }

    if (g_control_side_flag != 0)
    {
        side_control_timer(&g_control_time,
                           &g_control_flag,
                           &g_control_target,
                           &g_control_turn_manual_params,
                           &g_vel_motor,
                           &g_euler_angle_bias,
                           &g_control_motion_params);
    }

    control_shutdown(&g_control_target, &g_euler_angle_bias, &g_vel_motor);
}
