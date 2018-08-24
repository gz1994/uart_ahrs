#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"
#include "IOI2C.h"
#include "delay.h"
#include "LED.h"
#include "eeprom.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

extern int16_t  HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
		 HMC5883_minx,HMC5883_miny,HMC5883_minz;
extern unsigned char HMC5883_calib;

void HMC5883L_SetUp(void);	//初始化
void HMC58X3_getID(char id[3]);	//读芯片ID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC58X3_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Start_Calib(void);  //开始标定
void HMC5883L_Save_Calib(void);	  //结束标定 并保存标定值
#endif

//------------------End of File----------------------------
