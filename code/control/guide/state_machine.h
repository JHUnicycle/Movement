#ifndef _STATE_MACHINE_H
#define _STATE_MACHINE_H

// #include "zf_common_headfile.h"
// #include "image.h"
// #include "detection.h"

// typedef enum
// {
//     STATE_SEARCHING, // 状态：寻找目标
//     STATE_TURNING,   // 低速转向
//     STATE_TRACKING,  // 状态：加速靠近目标
//     STATE_CLOSE      // 接近目标减速
// } Run_State;

// typedef enum
// {
//     SINGLE_CAMERA, // 单摄像头模式
//     DUAL_CAMERA,   // 双摄像头模式
// } Camera_Mode;

// // 通过策略枚举
// typedef enum
// {
//     PASS_STRATEGY_CENTER, // 直接通过中心
//     PASS_STRATEGY_LEFT,   // 从左侧通过
//     PASS_STRATEGY_RIGHT   // 从右侧通过
// } Pass_Strategy;

// // 面积比较的容差
// #define AREA_TOLERANCE 10
// // 距离平方比较的容差
// #define ERROR_TOLERANCE 5

// #define abs(x) ((x) > 0 ? (x) : -(x))

// static inline int8 compare_components(const void *a, const void *b)
// {
//     const Component_Info *comp_a = (const Component_Info *)a;
//     const Component_Info *comp_b = (const Component_Info *)b;

//     // 第一关键字：面积（降序），带容差
//     int32 area_diff = (int32)comp_a->bbox.area - (int32)comp_b->bbox.area;
//     if (abs(area_diff) > AREA_TOLERANCE)
//     {
//         return (area_diff > 0) ? -1 : 1;
//     }

//     // 第二关键字：到中心的距离（升序），带容差
//     const int16 center_x = IMG_WIDTH / 2;
//     const int16 center_y = IMG_HEIGHT / 2;

//     int32 dx_a = comp_a->center.x - center_x;
//     int32 dx_b = comp_b->center.x - center_x;

//     int32 dist_sq_diff = (int32)dx_a - (int32)dx_b;
//     if (abs(dist_sq_diff) > ERROR_TOLERANCE)
//     {
//         return (dist_sq_diff < 0) ? -1 : 1;
//     }

//     // 第三关键字：前置摄像头优先于后置摄像头
//     return (comp_a->camera_id < comp_b->camera_id) ? -1 : 1;
// }

// void state_machine_init(Camera_Mode mode);

// void state_machine_imghandler(void);

// void state_machine_set_state(Run_State state);

// uint32 state_machine_get_component_count(void);

// Component_Info *state_machine_get_components(void);

// Run_State state_machine_get_state(void);

// uint8 *state_machine_get_front_img(void);

#endif
