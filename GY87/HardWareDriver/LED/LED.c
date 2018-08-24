/* KEY.C file
STM32-SDK 开发板相关例程
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-02-28
测试： 本程序已在第七实验室的STM32-SDK上完成测试
功能：实现	STM32-SDK 开发板上的两个 LED 操作接口

---------硬件上的引脚连接:----------
LED -->  PA8  	(输出低电平,灯亮;输出高电平灯灭)
------------------------------------
 */

#include "LED.h"


/**************************实现函数********************************************
*函数原型:		void Initial_PWMLED(void)
*功　　能:		配置 PWM  使得PA8输出PWM信号.
*******************************************************************************/
void Initial_PWMLED(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
 	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	TIM_DeInit(TIM1);
	
  	TIM_TimeBaseStructure.TIM_Period = 200;       
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;	   
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter =0;
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	   
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//输出极性 
  	TIM_OCInitStructure.TIM_Pulse = 20;	

  	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	//使能OC1 自动重载
  	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// 使能TIM1重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	//使能Time1 更新中断。即溢出中断		 
	//TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  	//启动定时器1
  	TIM_Cmd(TIM1, ENABLE);   
	TIM_CtrlPWMOutputs(TIM1, ENABLE);               
}

//LED 亮度级别表
static int LightLevel[40]={0,0,0,0,0,1,1,2,4,8,16,32,50,64,80,100,100,120,140,180,180,140,120,100,100,80,64,50,32,16,8,4,2,1,1,0,0,0,0,0};
u8 lightc=0;
/**************************实现函数********************************************
*函数原型:		void LED_Change(void)
*功　　能:		改变LED的亮度，从	LightLevel 数据
*******************************************************************************/
void LED_Change(void)
{
	TIM1->CCR1=LightLevel[lightc]; //更新通道1的比较值
	if(++lightc==40)lightc=0;
}

/**************************实现函数********************************************
*函数原型:		void LED_Reverse(void)
*功　　能:		LED 灯取反, 即,当亮时设置端口使之转成灭状态,
								当灭时设置端口使之转成亮状态.
*******************************************************************************/
void LED_Reverse(void)
{
	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8))	
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		else
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//------------------End of File----------------------------
