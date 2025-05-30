#ifndef _TEST_H_
#define _TEST_H_
#include "zf_common_headfile.h"

void test_bottom_motor();
void test_side_motor();
void test_attitude();
void test_imu();
void test_noise();
void test_side_deadzone();
void test_bottom_deadzone();
void test_double_camera();
void test_image();
void test_wireless_uart();
void test_key();
void test_sd_card(); // 添加SD卡测试函数声明
extern uint8 wireless_flag;
#endif