#include "diode.h"
#include "pin.h"

#include "zf_driver_gpio.h"

static const gpio_pin_enum DIODE_PTxn[DIODE_NUM] = {DIODE_LIST}; // LED/无源蜂鸣器引脚列表

static DIODE_INFO diode_info[DIODE_NUM];
static DIODE_STATUS_e diode_status[DIODE_NUM];
static uint8 diode_count[DIODE_NUM][2];

void diode_init()
{
    uint8 i = DIODE_NUM;
    while (i--)
    {
        gpio_init(DIODE_PTxn[i], GPO, GPIO_LOW, GPO_PUSH_PULL);
    }
}

void diode_set(DIODE_e i, uint8 on, uint8 pause, uint8 count)
{
    diode_info[i].diode_on_interrupts = on;
    diode_info[i].diode_pause_interrupts = pause;
    diode_info[i].diode_counts = count;
}

void diode_on(DIODE_e i)
{
    diode_status[i] = DIODE_ON;
    diode_count[i][0] = diode_count[i][1] = 0;
    gpio_set_level(DIODE_PTxn[i], DIODE_ON);
}

void diode_off(DIODE_e i)
{
    diode_status[i] = DIODE_OFF;
    diode_count[i][0] = diode_count[i][1] = 0;
    gpio_set_level(DIODE_PTxn[i], DIODE_OFF);
}

void diode_handler()
{
    uint8 i = DIODE_NUM;
    while (i--)
    {
        diode_count[i][0]++;
        switch (diode_status[i])
        {
        case DIODE_OFF:
            break;
        case DIODE_ON:
            if (diode_count[i][0] == diode_info[i].diode_on_interrupts)
            {
                diode_count[i][0] = 0;
                diode_count[i][1]++;
                diode_status[i] = DIODE_PAUSE;
                gpio_set_level(DIODE_PTxn[i], DIODE_OFF);
            }
            break;
        case DIODE_PAUSE:
            if (diode_count[i][0] == diode_info[i].diode_pause_interrupts)
            {
                diode_count[i][0] = 0;
                diode_status[i] = DIODE_ON;
                gpio_set_level(DIODE_PTxn[i], DIODE_ON);
            }
            break;
        }
        if (diode_count[i][1] == diode_info[i].diode_counts)
        {
            diode_count[i][1] = 0;
            diode_off(i);
        }
    }
}
