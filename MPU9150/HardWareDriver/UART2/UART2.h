#ifndef __UART2_H
#define __UART2_H

#include <stdio.h>
#include "stm32f10x.h"

#define Gyro_init  0xE0
#define HMC_calib  0xE1
#define High_init  0xE2
#define HMC_calib_begin  0xE3


void Initial_UART2(u32 baudrate);
void UART2_Put_Char(unsigned char DataToSend);
u8 UART2_Get_Char(void);
void UART2_Put_String(unsigned char *Str);
void UART2_Putc_Hex(uint8_t b);
void UART2_Putw_Hex(uint16_t w);
void UART2_Putdw_Hex(uint32_t dw);
void UART2_Putw_Dec(uint32_t w);
void UART2_Putint_Dec(int16_t in);
void UART2_Putintp_Dec(int16_t in);
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);

void UART2_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz
,int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec);

#endif

//------------------End of File----------------------------

