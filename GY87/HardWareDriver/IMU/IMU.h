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
	volatile float exInt, eyInt, ezInt;  // 误差积分
	volatile float q0, q1, q2, q3; // 全局四元数
	volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
};

extern float Kp;
extern float Ki;

//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态
uint32_t micros(void);	//读取系统上电后的时间  单位 us 

#endif

//------------------End of File----------------------------
