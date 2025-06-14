/*********************************************************************************************************************
 * TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK
 *接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC377 开源库的一部分
 *
 * TC377 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即
 *GNU通用公共许可证）的条款 即 GPL 的第3版（即
 *GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt
 *文件中 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu1_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC377TP
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/

#include "attitude.h"
#include "control.h"
#include "menu.h"
#include "system.h"
#include "velocity.h"
#include "receiver.h"
#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
void core1_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready(); // 等待所有核心初始化完毕

    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        if (g_exit_menu_flag && g_show_run_param_flag)
        {
            // lcd_show_string(0, 0, "Pitch:");
            // lcd_show_float(8, 0, currentFrontAngle, 3, 3);
            // lcd_show_string(0, 1, "CurrP:");
            // lcd_show_float(8, 1, currentFrontAngle - g_euler_angle_bias.pitch,
            //                3, 3);
            // lcd_show_float(0, 2, g_vel_motor.bottomFiltered, 3, 3);
            // lcd_show_string(0, 3, "FV:");
            // lcd_show_float(8, 3, g_control_target.bottom_angle, 3, 3);
            // lcd_show_string(0, 4, "FA:");
            // lcd_show_float(8, 4, g_control_target.bottom_angle_vel, 3, 3);
            // lcd_show_int(0, 6, get_bottom_duty(), 5);
            // lcd_show_float(0, 7, bottom_angle_velocity_PID.Ki, 3, 3);

            lcd_show_int(0, 0, g_received_vel, 5);
            lcd_show_string(0, 1, "BAV:");
            lcd_show_float(8, 1, bottom_angle_velocity_PID.Kp, 3, 3);
            lcd_show_string(0, 2, "BA:");
            lcd_show_float(8, 2, bottom_angle_PID.Kp, 3, 3);
            lcd_show_string(0, 3, "BV:");
            lcd_show_float(8, 3, bottom_velocity_PID.Kp, 3, 3);
            
            // lcd_show_float(0, 0, currentSideAngle, 3, 3);
            // lcd_show_float(0, 1, currentSideAngle - g_euler_angle_bias.roll,
            //                3, 3);
            // if (runState == CAR_RUNNING)
            // {
            //     lcd_show_string(0, 0, "RUNNING");
            // }
            // else if (runState == CAR_STOP)
            // {
            //     lcd_show_string(0, 0, "STOP");
            // }
            // else
            // {
            //     lcd_show_string(0, 0, "UNKNOWN");
            // }
            system_delay_ms(50);
            // lcd_clear(); // 清屏
        }
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
