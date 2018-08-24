/* main.c file

���ܣ�
1.��ʼ��������������
2.������̬����͸߶Ȳ���
3.���������̬�͸���������������ϴ��� MiniIMU AHRS �������
4.��Ӧ PC���͵�����
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

//�ϴ����ݵ�״̬��
#define REIMU  0x01 //�ϴ��������̬����
#define REMOV  0x02	//�ϴ������������
#define REHMC  0x03	//�ϴ������Ƶı궨ֵ

#define FAST_GAINS_THRESHOLD 15000

uint32_t Upload_Speed  = 30 ;  //�����ϴ��ٶ�  ��λ Hz
//#define upload_time (1000000/Upload_Speed)  //�����ϴ���ʱ�䡣��λΪus
uint32_t upload_time;  //�����ϴ���ʱ�䡣��λΪus

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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		int main(void)
*��������:		������
*******************************************************************************/
int main(void)
{
	u8 buf[200];
	int i;
	int16_t Math_hz=0;	 
	float ypr[3]; // yaw pitch roll
	int16_t angles[8];
	int kp_count=0;
	
	delay_init(72);		//��ʱ��ʼ��
	delay_ms(300);	//�ȴ������ϵ�
	
	Initial_PWMLED();
	Initial_UART1(115200);
	IIC_Init();	 //��ʼ��I2C�ӿ�
	
	IMU_init(); //��ʼ��IMU�ʹ�����
	system_micrsecond=micros();
	set_update_rate(Upload_Speed);

	//��ѭ��
	while(1)
	{	
		//delay_ms(1); //��ʱ����Ҫ����ô�졣
		IMU_getYawPitchRoll(ypr); //��̬����
		Math_hz++; //������� ++	
		
		MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if((gx>FAST_GAINS_THRESHOLD) || (gx<-FAST_GAINS_THRESHOLD) || 
				(gy>FAST_GAINS_THRESHOLD) || (gy<-FAST_GAINS_THRESHOLD) || 
				(gz>FAST_GAINS_THRESHOLD) || (gz<-FAST_GAINS_THRESHOLD))
		{
			kp_count = 0;
		}
	//-------------��λ��------------------------------
		//�Ƿ��˸��� ��λ����ʱ���ˣ�
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
				state = REMOV; //����״̬��
				break;
				case REMOV:
				MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
				HMC58X3_getlastValues(&hx,&hy,&hz);
				UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
				state = REIMU;
				if(HMC5883_calib)state = REHMC; //��Ҫ���͵�ǰ�����Ʊ궨ֵ
				break;
				default: 
				UART1_ReportHMC(HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
					HMC5883_minx,HMC5883_miny,HMC5883_minz,0);//���ͱ궨ֵ
				state = REIMU;
				break;
				}//switch(state) 			 
				system_micrsecond=micros();	 //ȡϵͳʱ�� ��λ us 
			}
			else
			{
			/* ����λ���ϴ����� */
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
				
				system_micrsecond=micros();	 //ȡϵͳʱ�� ��λ us 				
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
//							case Gyro_init:			MPU6050_InitGyro_Offset(); break; //��ȡ��������ƫ
//							case HMC_calib:			HMC5883L_Save_Calib();	break;   //��������Ʊ궨
//							case HMC_calib_begin:	HMC5883L_Start_Calib();	break; //��ʼ�����Ʊ궨	
						}
						UART1_Put_String("OK");
					}
					break;
				case 0x15:
					set_update_rate(u1_rx_buf[4]);//�����ϴ�����
					break;
				case 0xFF:
					update_ahrs = u1_rx_buf[4];					
					break;
				}
			}
			
			u1_rx_over = 0;
			u1_rx_length = 0;
		}// ����PC ���͵�����

	}//��ѭ�� while(1) ����

}  //main	


//------------------End of File----------------------------
