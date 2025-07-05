#ifndef _SWITCH_H
#define _SWITCH_H

#include "zf_common_headfile.h"

typedef enum
{
    SWITCH_NAME,
    SWITCH_NUM,
} SWITCH_e;

typedef enum
{
    SWITCH_OFF = 0, // 开关关闭时应为低
    SWITCH_ON = 1,  // 开关打开时应为高
} SWITCH_STATUS_e;

void switch_init(void);
SWITCH_STATUS_e switch_get_state(SWITCH_e state);

#endif