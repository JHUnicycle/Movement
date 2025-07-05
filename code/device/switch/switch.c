#include "switch.h"
#include "pin.h"
#include "zf_driver_gpio.h"

static const gpio_pin_enum SWITCH_PTxn[SWITCH_NUM] = {SWITCH_LIST}; // 按键引脚列表

void switch_init()
{
    uint8 i = SWITCH_NUM;
    while (i--)
    {
        gpio_init(SWITCH_PTxn[i], GPI, GPIO_LOW, GPO_PUSH_PULL);
    }
}

SWITCH_STATUS_e switch_get_state(SWITCH_e i)
{
    return (SWITCH_STATUS_e)(gpio_get_level(SWITCH_PTxn[i]));
}