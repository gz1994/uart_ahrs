#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"
#include "IOI2C.h"
#include "delay.h"
#include "eeprom.h"


extern int16_t  HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
		 HMC5883_minx,HMC5883_miny,HMC5883_minz;
extern unsigned char HMC5883_calib;

int16_t my_min(int16_t x,int16_t y);
int16_t my_max(int16_t x,int16_t y);

void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) ;
void HMC5883L_SetUp(void);	//初始化
void HMC58X3_getID(char id[3]);	//读芯片ID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC58X3_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Start_Calib(void);  //开始标定
void HMC5883L_Save_Calib(void);	  //结束标定 并保存标定值
#endif

//------------------End of File----------------------------
