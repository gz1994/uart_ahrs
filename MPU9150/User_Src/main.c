/* main.c file

���ܣ�
1.��ʼ��������������
2.������̬����͸߶Ȳ���
3.���������̬�͸���������������ϴ��� MiniIMU AHRS �������
4.��Ӧ PC���͵�����
------------------------------------
*/

#include "stm32f10x.h"
#include "UARTs.h"
#include "UART2.h"
#include "IOI2C.h"
#include "delay.h"
#include "MPU6050.h"
#include "IMU.h"
#include "eeprom.h"
#include "stdio.h"
#include "ak8975.h"

//�ϴ����ݵ�״̬��
#define REIMU  0x01 //�ϴ��������̬����
#define REMOV  0x02	//�ϴ������������
#define REHMC  0x03	//�ϴ������Ƶı궨ֵ

#define Upload_Speed  15   //�����ϴ��ٶ�  ��λ Hz
#define upload_time (1000000/Upload_Speed)/2  //�����ϴ���ʱ�䡣��λΪus

int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;

uint32_t system_micrsecond;
int led_time=0;

float ypr[3]; // yaw pitch roll
int16_t Math_hz=0;
u8 state= REIMU;
u8 update_flag=0;

void timer4_init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 72-1;
	TIM_TimeBaseStructure.TIM_Prescaler =5000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

void led_on(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

void led_off(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

void led_light(int time)
{
	led_time = time;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		int main(void)
*��������:		������
*******************************************************************************/
int main(void)
{	
	unsigned char PC_comm; //PC ����ؼ��ֽ�	 
	u8 buffer[200];
	float mag_value[3];
	float Mxh,Myh;
	
	/* ����ϵͳʱ��Ϊ72M ʹ���ⲿ8M����+PLL*/      
    SystemInit();
	
	/* USE SWD Mode */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	delay_init(72);		//��ʱ��ʼ��
	Initial_UART1(115200L);
	//Initial_UART2(115200L);
	timer4_init();
	IIC_Init();	 //��ʼ��I2C�ӿ�
	delay_ms(300);	//�ȴ������ϵ�

	IMU_init(); //��ʼ��IMU�ʹ�����
	led_init();
	system_micrsecond=micros();

	while(1)//��ѭ��	
	{			
		if(update_flag)
		{		
			led_on();
			IMU_getYawPitchRoll(ypr); //��̬����
			Math_hz++; //������� ++s						
			led_off();
//			/* get yaw *///----------------------------------------
//			AK8975_getRaw(&hx,&hy,&hz);
//			
//			mag_value[0] = (float)hx;
//			mag_value[1] = (float)hy;
//			mag_value[2] = (float)hz;			
////			Mxh = mag_value[0]*cos(ypr[1]) + mag_value[1]*sin(ypr[0])*sin(ypr[1]) + mag_value[2]*cos(ypr[2])*sin(ypr[1]);
////			Myh = mag_value[1]*cos(ypr[2])*sin(ypr[2]);			
//			Mxh = mag_value[0];
//			Myh = mag_value[1];
//			
//			if(Mxh>0) {
//				if(Myh>0) {
//					ypr[0] = ((atan2(Myh,Mxh)*180)/M_PI);
//				}
//				else {
//					ypr[0] = 360+((atan2(Myh,Mxh)*180)/M_PI);
//				}
//			}
//			else if(Mxh==0) {
////				if(Myh>0)  ypr[0] = 90;
////				if(Myh<0)  ypr[0] = 270;
//			}
//			else {
//				if(Myh>0) {
//					ypr[0] = ((atan2(Myh,Mxh)*180)/M_PI);
//				}
//				else {
//					ypr[0] = 360+((atan2(Myh,Mxh)*180)/M_PI);
//				}
//			}
			//------------------------------------------------------
			
			update_flag = 0;			
		}
	    //-------------��λ��------------------------------
		//�Ƿ��˸��� ��λ����ʱ���ˣ�
		if((micros()-system_micrsecond)>upload_time)
		{
			if(1)
			{
				switch(state)
				{ 
				case REIMU:
					UART1_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
					(int16_t)(ypr[2]*10.0),0,0,0,Math_hz*16);
					Math_hz=0;
					state = REMOV; //����״̬��
					break;
				case REMOV:
					MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
					AK8975_get_values(&hx,&hy,&hz);

					UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
					state = REIMU;
					if(AK8975_calib)state = REHMC; //��Ҫ���͵�ǰ�����Ʊ궨ֵ
					break;
				default: 
					UART1_ReportHMC(AK8975_minx,AK8975_miny,AK8975_minz,
						AK8975_maxx,AK8975_maxy,AK8975_maxz,0);//���ͱ궨ֵ
					state = REIMU;
					break;
				}//switch(state) 
			}	
			else			
			{
				sprintf(buffer,"yaw:%f\thx=%d\thy=%d\thz=%d\thx_max/min:%d/%d\thy_min/max:%d/%d\thz_min/max:%d/%d\r\n",
								ypr[0],hx,hy,hz,AK8975_minx,AK8975_maxx,AK8975_miny,AK8975_maxy,AK8975_minz,AK8975_maxz);
				UART1_Put_String(buffer);
		
			}
			

			system_micrsecond=micros();	 //ȡϵͳʱ�� ��λ us 
		}
		//--------------------------------------------------
		//����PC������������
		if((PC_comm=UART1_CommandRoute())!=0xff)
		{
			//��������ʶ
			switch(PC_comm)
			{ 
				case Gyro_init:			
					MPU6050_InitGyro_Offset();
					led_light(100);
					break; //��ȡ��������ƫ
				case HMC_calib:			
					AK8975_Save_Calib();
					led_light(100);
					break;   //��������Ʊ궨
				case High_init:			 	
					break;
				case HMC_calib_begin:	
					AK8975_Start_Calib();
					led_light(100);
					break; //��ʼ�����Ʊ궨
			}
		}// ����PC ���͵�����

	}//��ѭ�� while(1) ����

}  //main	


/**
  * @brief  This function handles TIM2 Handler.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)  
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{
		update_flag = 1;
		if(led_time>0)
		{
			led_time--;
			led_on();
		}
		else
		{
			led_off();
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  ); 		
	}
}

//------------------End of File----------------------------
