#include "inout.h"

#define TRIGGER_GPIO 	GPIOB
#define TRIGGER_PIN 	GPIO_Pin_8

#define RELAY_GPIO 		GPIOB
#define RELAY_PIN 		GPIO_Pin_5

static u8 relay_enable=RELAY_ENABLE;
static uint32_t relay_micros; 
static u8 relay_opt_stat;
static uint32_t relay_pwm = 10000;
static u8 relay_flag = RELAY_OFF;

void trigger_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = TRIGGER_PIN;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(TRIGGER_GPIO, &GPIO_InitStructure); 
}

u8 trigger_read(void)
{
	if(0 ==  GPIO_ReadInputDataBit(TRIGGER_GPIO,TRIGGER_PIN)) {
		return TRIGGER_ON;
	} else {
		return TRIGGER_OFF;
	}
}

void relay_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = RELAY_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(RELAY_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(RELAY_GPIO, RELAY_PIN);
	
}

/**/
void relay_switch(u8 flag)
{
	relay_flag = flag;
}

/**/
void relay_set_enable(u8 n)
{
	if(n==RELAY_ENABLE){
		relay_enable = RELAY_ENABLE;
	}
	else {
		relay_enable=RELAY_DISABLE;
	}
}

void relay_set_pwm(u8 pwm)
{
	if(pwm>99){
		return;
	}
	relay_pwm = pwm *1000;
}

void relay_output(u8 n)
{
	if(n==1)
	{
		relay_opt_stat = RELAY_ON;
		GPIO_ResetBits(RELAY_GPIO, RELAY_PIN);//relay on		
	}
	else
	{
		relay_opt_stat = RELAY_OFF;
		GPIO_SetBits(RELAY_GPIO,RELAY_PIN);//relay off
	}
}

/* relay callback */
void relay_callback(uint32_t t)
{
	if((relay_enable == RELAY_ENABLE) && (relay_flag == RELAY_ON)) 
	{		
		if(relay_opt_stat == RELAY_ON) {
			if((t - relay_micros) > relay_pwm) {
				relay_output(0);
				relay_micros = t;
			}
		} else {
			if((t - relay_micros) > (100000 - relay_pwm)) {
				relay_output(1);
				relay_micros = t;
			}
		}				
	}
	else
	{
		relay_output(0);
		relay_micros = t;
	}
}







