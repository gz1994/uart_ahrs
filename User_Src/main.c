/* main.c file

功能：
1.初始化各个传感器，
2.运行姿态解算和高度测量
3.将解算的姿态和各个传感器的输出上传到 MiniIMU AHRS 测试软件
4.响应 PC发送的命令
------------------------------------
*/

#include "stm32f10x.h"
#include "LED.h"
#include "UARTs.h"
#include "IOI2C.h"
#include "delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "IMU.h"
#include "eeprom.h"
#include "stdio.h"
#include "string.h"

#define VERSION_MAIN  0
#define VERSION_SUB   1

//上传数据的状态机
#define REIMU  0x01 //上传解算的姿态数据
#define REMOV  0x02	//上传传感器的输出
#define REHMC  0x03	//上传磁力计的标定值

#define FAST_GAINS_THRESHOLD 15000

uint32_t Upload_Speed  = 30 ;  //数据上传速度  单位 Hz
//#define upload_time (1000000/Upload_Speed)  //计算上传的时间。单位为us
uint32_t upload_time;  //计算上传的时间。单位为us

int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;
int32_t Temperature = 0, Pressure = 0, Altitude = 0;
uint32_t system_micrsecond;
int16_t hmcvalue[3];
u8 state= REIMU;

static u8 update_ahrs = 1;

void set_update_rate(uint32_t speed)
{
	if(speed>60)
	{
		speed = 60;
	}
	Upload_Speed = speed;
	upload_time = (1000000/speed);
}

/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{
	u8 buf[200];
	int i;
	int16_t Math_hz=0;	 
	float ypr[3]; // yaw pitch roll
	int16_t angles[8];
	int kp_count=0;
	
	delay_init(72);		//延时初始化
	delay_ms(300);	//等待器件上电
	
	Initial_PWMLED();
	Initial_UART1(115200);
	IIC_Init();	 //初始化I2C接口
	
	IMU_init(); //初始化IMU和传感器
	system_micrsecond=micros();
	set_update_rate(Upload_Speed);

	//主循环
	while(1)
	{	
		//delay_ms(1); //延时，不要算那么快。
		IMU_getYawPitchRoll(ypr); //姿态更新
		Math_hz++; //解算次数 ++	
		
		MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if((gx>FAST_GAINS_THRESHOLD) || (gx<-FAST_GAINS_THRESHOLD) || 
				(gy>FAST_GAINS_THRESHOLD) || (gy<-FAST_GAINS_THRESHOLD) || 
				(gz>FAST_GAINS_THRESHOLD) || (gz<-FAST_GAINS_THRESHOLD))
		{
			kp_count = 0;
		}
	//-------------上位机------------------------------
		//是否到了更新 上位机的时间了？
		if((micros()-system_micrsecond)>upload_time)
		{
			if(kp_count < (Upload_Speed*3)) {
				Kp = 6.0f;
				kp_count++;
			}
			else if(kp_count == (Upload_Speed*3)) {
				kp_count++;
				Kp = 2.0f;
			}
			
			if(update_ahrs==1)
			{
				switch(state){ 
				case REIMU:
				UART1_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
					(int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10,Math_hz*16);
				Math_hz=0;
				state = REMOV; //更改状态。
				break;
				case REMOV:
				MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
				HMC58X3_getlastValues(&hx,&hy,&hz);
				UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
				state = REIMU;
				if(HMC5883_calib)state = REHMC; //需要发送当前磁力计标定值
				break;
				default: 
				UART1_ReportHMC(HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
					HMC5883_minx,HMC5883_miny,HMC5883_minz,0);//发送标定值
				state = REIMU;
				break;
				}//switch(state) 			 
				system_micrsecond=micros();	 //取系统时间 单位 us 
			}
			else
			{
			/* 向上位机上传数据 */
				angles[0] = (int16_t)((ypr[0]+180) * 10);
				angles[1] = (int16_t)((ypr[1]+180) * 10);
				angles[2] = (int16_t)((ypr[2]+180) * 10);			
			
				buf[0] = 0x7E;
				buf[1] = 0x7E;
				buf[2] = 0x0E;
				buf[3] = 0x01;
				buf[4] = angles[0]>>8;
				buf[5] = angles[0]&0xFF;
				buf[6] = angles[1]>>8;
				buf[7] = angles[1]&0xFF;
				buf[8] = angles[2]>>8;
				buf[9] = angles[2]&0xFF;
				buf[10] = 0;
				for(i=0;i<9;i++)
				{
					buf[11] += buf[i+2];
				}
				buf[12] = 0x0D;
				buf[13] = 0x0A;
				
				system_micrsecond=micros();	 //取系统时间 单位 us 				
			}
			
			LED_Change();
		}
		
		if(u1_rx_over)
		{
			if(u1_rx_buf[0]==0x7E)
			{
				switch(u1_rx_buf[3])
				{
				case 0x00:
					UART1_Put_Char(VERSION_MAIN);
					UART1_Put_Char(VERSION_SUB);
					break;
				case 0x11:		
					break;
				case 0x14:
					{
						switch(u1_rx_buf[4])
						{
//							case Gyro_init:			MPU6050_InitGyro_Offset(); break; //读取陀螺仪零偏
//							case HMC_calib:			HMC5883L_Save_Calib();	break;   //保存磁力计标定
//							case HMC_calib_begin:	HMC5883L_Start_Calib();	break; //开始磁力计标定	
						}
						UART1_Put_String("OK");
					}
					break;
				case 0x15:
					set_update_rate(u1_rx_buf[4]);//设置上传速率
					break;
				case 0xFF:
					update_ahrs = u1_rx_buf[4];					
					break;
				}
			}
			
			u1_rx_over = 0;
			u1_rx_length = 0;
		}// 处理PC 发送的命令

	}//主循环 while(1) 结束

}  //main	


//------------------End of File----------------------------
