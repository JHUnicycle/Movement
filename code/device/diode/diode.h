#ifndef _DIODE_h_
#define _DIODE_h_

#include "zf_common_headfile.h"
#include "pin.h"

typedef enum
{
    DIODE_NAME,
    DIODE_NUM,
} DIODE_e;

typedef struct
{
    uint8 diode_on_interrupts, diode_pause_interrupts, diode_counts;
} DIODE_INFO;

typedef enum
{
    DIODE_PAUSE = -1,
    DIODE_OFF = 0,
    DIODE_ON = 1,
} DIODE_STATUS_e;

void diode_init(void);
void diode_set(DIODE_e, uint8, uint8, uint8);
void diode_on(DIODE_e);
void diode_off(DIODE_e);
void diode_handler(void);

#endif
