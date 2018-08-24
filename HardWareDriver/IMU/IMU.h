#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "UARTs.h"
#include "delay.h"
#include <math.h>

#define M_PI  (float)3.1415926535
struct AHRS_PARAM
{
	volatile float exInt, eyInt, ezInt;  // ������
	volatile float q0, q1, q2, q3; // ȫ����Ԫ��
	volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
};

extern float Kp;
extern float Ki;

//Mini IMU AHRS �����API
void IMU_init(void); //��ʼ��
void IMU_getYawPitchRoll(float * ypr); //������̬
uint32_t micros(void);	//��ȡϵͳ�ϵ���ʱ��  ��λ us 

#endif

//------------------End of File----------------------------
