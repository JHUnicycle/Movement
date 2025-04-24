#include "test.h"
#include "attitude.h"
#include "encoder.h"
#include "lcd.h"
#include "menu_input.h"
#include "motor.h"
#include "small_driver_uart_control.h"
#include "velocity.h"
#include "zf_common_headfile.h"

#define IMG_SIZE_W MT9V03X_W / 2
#define IMG_SIZE_H MT9V03X_H / 2

void test_bottom_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: forward");
    lcd_show_string(0, 1, "KEY_D: backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    while (keymsg.key != KEY_L) {
        if (keymsg.key == KEY_U)  // 向前
        {
            gpio_set_level(DIR_BOTTOM, 1);
            pwm_set_duty(MOTOR_BOTTOM, 8000);
        }
        if (keymsg.key == KEY_D)  // 向后
        {
            gpio_set_level(DIR_BOTTOM, 0);
            pwm_set_duty(MOTOR_BOTTOM, 8000);
        }
        lcd_show_int(0, 5, g_vel_motor.bottom, 5);
        lcd_show_float(0, 6, g_vel_motor.bottomReal, 5, 5);
        lcd_show_float(0, 7, g_vel_motor.bottomFiltered, 5, 5);
        // encoder_clear_count(ENCODER_BOTTOM);
    }
    pwm_set_duty(MOTOR_BOTTOM, 0);
    lcd_clear();
}

void test_side_motor() {
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: left  forward");
    lcd_show_string(0, 1, "KEY_D: left backward");
    lcd_show_string(0, 2, "KEY_B: right forward");
    lcd_show_string(0, 3, "KEY_R:right backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    // int count = 0;
    while (keymsg.key != KEY_L) {
        if (keymsg.key == KEY_U)  // 向前
        {
            small_driver_set_duty(2000, 0);
        }
        if (keymsg.key == KEY_D)  // 向后
        {
            small_driver_set_duty(-2000, 0);
        }
        if (keymsg.key == KEY_B)  // 向前
        {
            small_driver_set_duty(0, 2000);
        }
        if (keymsg.key == KEY_R)  // 向后
        {
            small_driver_set_duty(0, -2000);
        }
        // count++;
        lcd_show_string(0, 5, "Front:");
        lcd_show_int(8, 5, g_vel_motor.momentumFront, 5);
        lcd_show_string(0, 6, "Back:");
        lcd_show_int(8, 6, g_vel_motor.momentumBack, 5);
        // lcd_show_string(0, 7, "count:");
        // lcd_show_int(8, 7, count, 5);
    }
    small_driver_set_duty(0, 0);
    lcd_clear();
}

// void test_side_motor() {
//     lcd_clear();
//     lcd_show_string(0, 0, "KEY_U: left  forward");
//     lcd_show_string(0, 1, "KEY_D: left backward");
//     lcd_show_string(0, 2, "KEY_B: right forward");
//     lcd_show_string(0, 3, "KEY_R:right backward");
//     lcd_show_string(0, 4, "Press KEY_L to exit");
//     int count = 0;
//     while (keymsg.key != KEY_L) {
//         if (keymsg.key == KEY_U) {
//             small_driver_set_duty(-600, 600);
//         } else if (keymsg.key == KEY_D) {
//             small_driver_set_duty(-5000, 5000);
//         }
//     }
//     small_driver_set_duty(0, 0);
//     lcd_clear();
// }

void test_attitude() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        lcd_show_string(0, 0, "Pitch:");
        lcd_show_float(0, 1, currentFrontAngle, 3, 3);
        lcd_show_string(0, 2, "Row:");
        lcd_show_float(0, 3, currentSideAngle, 3, 3);
        lcd_show_string(0, 4, "Yaw:");
        lcd_show_float(0, 5, yawAngle, 3, 3);
    }
    lcd_clear();
}

void test_imu() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        lcd_show_string(0, 0, "x:");
        lcd_show_float(0, 1, g_imu_data.gyro.x, 3, 3);
        lcd_show_float(8, 1, g_imu_data.acc.x, 3, 3);
        lcd_show_string(0, 2, "y:");
        lcd_show_float(0, 3, g_imu_data.gyro.y, 3, 3);
        lcd_show_float(8, 3, g_imu_data.acc.y, 3, 3);
        lcd_show_string(0, 4, "z:");
        lcd_show_float(0, 5, g_imu_data.gyro.z, 3, 3);
        lcd_show_float(8, 5, g_imu_data.acc.z, 3, 3);
    }
    lcd_clear();
}

void test_noise() {
    lcd_clear();

    float sum_ax = 0.0f, sum_ax2 = 0.0f;
    float sum_ay = 0.0f, sum_ay2 = 0.0f;
    float sum_az = 0.0f, sum_az2 = 0.0f;
    float sum_gx = 0.0f, sum_gx2 = 0.0f;
    float sum_gy = 0.0f, sum_gy2 = 0.0f;
    float sum_gz = 0.0f, sum_gz2 = 0.0f;

    float var_ax = 0.0f;
    float var_ay = 0.0f;
    float var_az = 0.0f;
    float var_gx = 0.0f;
    float var_gy = 0.0f;
    float var_gz = 0.0f;

    uint8 T = 10;
    uint8 cnt = 0;
    uint32 total = 0;
    while (keymsg.key != KEY_L) {
        if (cnt < T) {
            // 读取传感器数据
            imu660rb_get_acc();
            imu660rb_get_gyro();

            // 实时计算并累加
            float current_ax =
                imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
            float current_ay =
                imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;
            float current_az =
                imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;

            float current_gx =
                imu660rb_gyro_transition(imu660rb_gyro_x) * DEG2RAD;
            float current_gy =
                imu660rb_gyro_transition(imu660rb_gyro_y) * DEG2RAD;
            float current_gz =
                imu660rb_gyro_transition(imu660rb_gyro_z) * DEG2RAD;

            // 累加原始值和平方值
            sum_ax += current_ax;
            sum_ax2 += current_ax * current_ax;
            sum_ay += current_ay;
            sum_ay2 += current_ay * current_ay;
            sum_az += current_az;
            sum_az2 += current_az * current_az;

            sum_gx += current_gx;
            sum_gx2 += current_gx * current_gx;
            sum_gy += current_gy;
            sum_gy2 += current_gy * current_gy;
            sum_gz += current_gz;
            sum_gz2 += current_gz * current_gz;
        } else {
            // 计算均值
            float mean_ax = sum_ax / total;
            float mean_ay = sum_ay / total;
            float mean_az = sum_az / total;
            float mean_gx = sum_gx / total;
            float mean_gy = sum_gy / total;
            float mean_gz = sum_gz / total;

            var_ax = (sum_ax2 / total) - (mean_ax * mean_ax);
            var_ay = (sum_ay2 / total) - (mean_ay * mean_ay);
            var_az = (sum_az2 / total) - (mean_az * mean_az);
            var_gx = (sum_gx2 / total) - (mean_gx * mean_gx);
            var_gy = (sum_gy2 / total) - (mean_gy * mean_gy);
            var_gz = (sum_gz2 / total) - (mean_gz * mean_gz);

            // lcd_show_string(0, 0, "acc_x:");
            // lcd_show_float(8, 0, var_ax, 5, 3);
            // lcd_show_string(0, 1, "acc_y:");
            // lcd_show_float(8, 1, var_ay, 5, 3);
            // lcd_show_string(0, 2, "acc_z:");
            // lcd_show_float(8, 2, var_az, 5, 3);
            // lcd_show_string(0, 3, "gyro_x:");
            // lcd_show_float(8, 3, var_gx, 5, 3);
            // lcd_show_string(0, 4, "gyro_y:");
            // lcd_show_float(8, 4, var_gy, 5, 3);
            // lcd_show_string(0, 5, "gyro_z:");
            // lcd_show_float(8, 5, var_gz, 5, 3);
            printf(
                "total: %d, acc_x: %f, acc_y: %f, acc_z: %f, gyro_x: %f, "
                "gyro_y: %f, gyro_z: %f\n",
                total, var_ax, var_ay, var_az, var_gx, var_gy, var_gz);
            cnt = 0;
        }
        cnt++;
        total++;
        system_delay_ms(100);
    }
    lcd_clear();
}

void test_camera() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        if (mt9v03x_finish_flag) {
            mt9v03x_finish_flag = 0;
            // edge_detect_dynamic(mt9v03x_image, edge_map, &edge_cfg);
            tft180_show_gray_image(0, 0, mt9v03x_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
            tft180_show_gray_image(tft180_width_max - IMG_SIZE_W, 0,
                                   mt9v03x_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
        }
        if (mt9v03x2_finish_flag) {
            mt9v03x2_finish_flag = 0;
            // edge_detect_dynamic(mt9v03x2_image, edge_map2, &edge_cfg);
            tft180_show_gray_image(0, tft180_height_max - IMG_SIZE_H,
                                   mt9v03x2_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
            tft180_show_gray_image(tft180_width_max - IMG_SIZE_W,
                                   tft180_height_max - IMG_SIZE_H,
                                   mt9v03x2_image, MT9V03X2_W, MT9V03X2_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
        }
    }
}