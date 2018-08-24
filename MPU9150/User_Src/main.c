/* main.c file

功能：
1.初始化各个传感器，
2.运行姿态解算和高度测量
3.将解算的姿态和各个传感器的输出上传到 MiniIMU AHRS 测试软件
4.响应 PC发送的命令
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

//上传数据的状态机
#define REIMU  0x01 //上传解算的姿态数据
#define REMOV  0x02	//上传传感器的输出
#define REHMC  0x03	//上传磁力计的标定值

#define Upload_Speed  15   //数据上传速度  单位 Hz
#define upload_time (1000000/Upload_Speed)/2  //计算上传的时间。单位为us

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

/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{	
	unsigned char PC_comm; //PC 命令关键字节	 
	u8 buffer[200];
	float mag_value[3];
	float Mxh,Myh;
	
	/* 配置系统时钟为72M 使用外部8M晶体+PLL*/      
    SystemInit();
	
	/* USE SWD Mode */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	delay_init(72);		//延时初始化
	Initial_UART1(115200L);
	//Initial_UART2(115200L);
	timer4_init();
	IIC_Init();	 //初始化I2C接口
	delay_ms(300);	//等待器件上电

	IMU_init(); //初始化IMU和传感器
	led_init();
	system_micrsecond=micros();

	while(1)//主循环	
	{			
		if(update_flag)
		{		
			led_on();
			IMU_getYawPitchRoll(ypr); //姿态更新
			Math_hz++; //解算次数 ++s						
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
	    //-------------上位机------------------------------
		//是否到了更新 上位机的时间了？
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
					state = REMOV; //更改状态。
					break;
				case REMOV:
					MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
					AK8975_get_values(&hx,&hy,&hz);

					UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
					state = REIMU;
					if(AK8975_calib)state = REHMC; //需要发送当前磁力计标定值
					break;
				default: 
					UART1_ReportHMC(AK8975_minx,AK8975_miny,AK8975_minz,
						AK8975_maxx,AK8975_maxy,AK8975_maxz,0);//发送标定值
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
			

			system_micrsecond=micros();	 //取系统时间 单位 us 
		}
		//--------------------------------------------------
		//处理PC发送来的命令
		if((PC_comm=UART1_CommandRoute())!=0xff)
		{
			//检查命令标识
			switch(PC_comm)
			{ 
				case Gyro_init:			
					MPU6050_InitGyro_Offset();
					led_light(100);
					break; //读取陀螺仪零偏
				case HMC_calib:			
					AK8975_Save_Calib();
					led_light(100);
					break;   //保存磁力计标定
				case High_init:			 	
					break;
				case HMC_calib_begin:	
					AK8975_Start_Calib();
					led_light(100);
					break; //开始磁力计标定
			}
		}// 处理PC 发送的命令

	}//主循环 while(1) 结束

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
