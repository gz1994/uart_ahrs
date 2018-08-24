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
*����ԭ��:		void Initial_PWMLED(void)
*��������:		���� PWM  ʹ��PA8���PWM�ź�.
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
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//������� 
  	TIM_OCInitStructure.TIM_Pulse = 20;	

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
	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8))	
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		else
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//------------------End of File----------------------------
