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
*函数原型:		void Initial_LED_GPIO(void)
*功　　能:		配置 LED 对应的端口为输出
*******************************************************************************/
void Initial_LED_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //使能GPIOA 的时钟,
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
  //配置PA8 为推挽输出  刷新频率为2Mhz
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  //应用配置到GPIOA 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  	/*
	配置 PA0  PA1 为 输入且使能上拉电阻
	*/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//应用配置　到GPIOA　
	GPIO_Init(GPIOA, &GPIO_InitStructure);


  //设置LED 端口输出高电平, 关灯.
  GPIO_SetBits(GPIOA, GPIO_Pin_8);	 
}

/**************************实现函数********************************************
*函数原型:		void Initial_PWMLED(void)
*功　　能:		配置 PWM  使得PB1输出PWM信号.
*******************************************************************************/
void Initial_PWMLED(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//使能TIMER3 的时钟信号
	//PCLK1经过2倍频后作为TIM3的时钟源等于36MHz
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
  	//使能 GPIOA 的时钟信号
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  	//配置 PA15 为复用推挽输出 刷新频率50MHz
 	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //应用配置
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 定时器配置:
	1.设置定时器最大计数值 256	
	2.设置时钟分频系数：TIM_CKD_DIV2
	3. 设置预分频：20，36M/2/20=900Khz  PWM的频率:900Khz/256 约3.5KHz
	4.定时器计数模式  向上计数模式
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 200;       
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;	   
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter =0;
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  	/* 配置PWM输出通道 3 
	1. 配置为PWM模式1  TIM3_CNT>TIM3_CCR3时引脚输出为低，否则为高
	2.使能PWM输出
	3. 设置占空比, 为 20/256
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	   
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//输出极性 
  	TIM_OCInitStructure.TIM_Pulse = 20;	
	//应用配置到Tim1 OC1
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
	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15))	
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);
		else
		GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

//------------------End of File----------------------------
