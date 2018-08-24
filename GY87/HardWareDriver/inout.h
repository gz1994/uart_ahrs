#ifndef __INOUT_H__
#define __INOUT_H__

#include "stm32f10x.h"

#define TRIGGER_ON  1
#define TRIGGER_OFF 0

#define RELAY_ON 	1
#define RELAY_OFF   0

#define RELAY_ENABLE 1
#define RELAY_DISABLE 0

void trigger_init(void);
u8 trigger_read(void);
void relay_init(void);
void relay_set_enable(u8 n);
void relay_switch(u8 flag);
void relay_set_pwm(u8 pwm);
void relay_callback(uint32_t t);

#endif







