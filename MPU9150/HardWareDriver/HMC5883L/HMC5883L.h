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
void HMC5883L_SetUp(void);	//��ʼ��
void HMC58X3_getID(char id[3]);	//��оƬID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //��ADC
void HMC58X3_mgetValues(float *arry); //IMU ר�õĶ�ȡ������ֵ
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Start_Calib(void);  //��ʼ�궨
void HMC5883L_Save_Calib(void);	  //�����궨 ������궨ֵ
#endif

//------------------End of File----------------------------
