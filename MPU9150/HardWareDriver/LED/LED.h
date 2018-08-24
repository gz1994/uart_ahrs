#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

//LEDs 定义LED 操作宏.
//(输出低电平,灯亮;输出高电平灯灭)
#define LED_ON()   GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define LED_OFF()  GPIO_SetBits(GPIOA, GPIO_Pin_8)



void Initial_LED_GPIO(void);
void LED_Reverse(void);
void Initial_PWMLED(void);
void LED_Change(void);

#endif


//------------------End of File----------------------------
