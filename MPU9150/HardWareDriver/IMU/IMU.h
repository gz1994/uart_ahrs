#ifndef __IMU_H
#define __IMU_H

#include "MPU6050.h"
#include "ak8975.h"
#include "UARTs.h"
#include "delay.h"

#include <math.h>
#define M_PI  (float)3.1415926535
#define USE_MAG 1

//Mini IMU AHRS �����API
void IMU_init(void); //��ʼ��
void IMU_getYawPitchRoll(float * ypr); //������̬
uint32_t micros(void);	//��ȡϵͳ�ϵ���ʱ��  ��λ us 

#endif

//------------------End of File----------------------------
