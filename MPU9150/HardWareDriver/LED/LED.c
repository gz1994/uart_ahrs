/* KEY.C file
STM32-SDK �������������
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-02-28
���ԣ� ���������ڵ���ʵ���ҵ�STM32-SDK����ɲ���
���ܣ�ʵ��	STM32-SDK �������ϵ����� LED �����ӿ�

---------Ӳ���ϵ���������:----------
LED -->  PA8  	(����͵�ƽ,����;����ߵ�ƽ����)
------------------------------------
 */

#include "LED.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_LED_GPIO(void)
*��������:		���� LED ��Ӧ�Ķ˿�Ϊ���
*******************************************************************************/
void Initial_LED_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //ʹ��GPIOA ��ʱ��,
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
  //����PA8 Ϊ�������  ˢ��Ƶ��Ϊ2Mhz
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  //Ӧ�����õ�GPIOA 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  	/*
	���� PA0  PA1 Ϊ ������ʹ����������
	*/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	//Ӧ�����á���GPIOA��
	GPIO_Init(GPIOA, &GPIO_InitStructure);


  //����LED �˿�����ߵ�ƽ, �ص�.
  GPIO_SetBits(GPIOA, GPIO_Pin_8);	 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_PWMLED(void)
*��������:		���� PWM  ʹ��PB1���PWM�ź�.
*******************************************************************************/
void Initial_PWMLED(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��TIMER3 ��ʱ���ź�
	//PCLK1����2��Ƶ����ΪTIM3��ʱ��Դ����36MHz
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
  	//ʹ�� GPIOA ��ʱ���ź�
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  	//���� PA15 Ϊ����������� ˢ��Ƶ��50MHz
 	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //Ӧ������
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��ʱ������:
	1.���ö�ʱ��������ֵ 256	
	2.����ʱ�ӷ�Ƶϵ����TIM_CKD_DIV2
	3. ����Ԥ��Ƶ��20��36M/2/20=900Khz  PWM��Ƶ��:900Khz/256 Լ3.5KHz
	4.��ʱ������ģʽ  ���ϼ���ģʽ
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 200;       
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;	   
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter =0;
	//Ӧ�����õ�TIM3 
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  	/* ����PWM���ͨ�� 3 
	1. ����ΪPWMģʽ1  TIM3_CNT>TIM3_CCR3ʱ�������Ϊ�ͣ�����Ϊ��
	2.ʹ��PWM���
	3. ����ռ�ձ�, Ϊ 20/256
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	   
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//������� 
  	TIM_OCInitStructure.TIM_Pulse = 20;	
	//Ӧ�����õ�Tim1 OC1
  	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	//ʹ��OC1 �Զ�����
  	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// ʹ��TIM1���ؼĴ���ARR
  	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	//ʹ��Time1 �����жϡ�������ж�		 
	//TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  	//������ʱ��1
  	TIM_Cmd(TIM1, ENABLE);   
	TIM_CtrlPWMOutputs(TIM1, ENABLE);               
}

//LED ���ȼ����
static int LightLevel[40]={0,0,0,0,0,1,1,2,4,8,16,32,50,64,80,100,100,120,140,180,180,140,120,100,100,80,64,50,32,16,8,4,2,1,1,0,0,0,0,0};
u8 lightc=0;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void LED_Change(void)
*��������:		�ı�LED�����ȣ���	LightLevel ����
*******************************************************************************/
void LED_Change(void)
{
	TIM1->CCR1=LightLevel[lightc]; //����ͨ��1�ıȽ�ֵ
	if(++lightc==40)lightc=0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void LED_Reverse(void)
*��������:		LED ��ȡ��, ��,����ʱ���ö˿�ʹ֮ת����״̬,
								����ʱ���ö˿�ʹ֮ת����״̬.
*******************************************************************************/
void LED_Reverse(void)
{
	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15))	
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);
		else
		GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

//------------------End of File----------------------------
