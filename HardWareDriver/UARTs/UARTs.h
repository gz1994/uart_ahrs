#ifndef __UARTS_H
#define __UARTS_H

#include <stdio.h>
#include "stm32f10x.h"

extern volatile unsigned char u1_rx_buf[200];
extern volatile unsigned char u1_rx_length;
extern volatile u8 u1_rx_over;

void Initial_UART1(u32 baudrate);
void UART1_Put_Char(unsigned char DataToSend);
u8 UART1_Get_Char(void);
void UART1_send_buf(u8 * buf, int len);
void UART1_Put_String(unsigned char *Str);
void UART1_Putc_Hex(uint8_t b);
void UART1_Putw_Hex(uint16_t w);
void UART1_Putdw_Hex(uint32_t dw);
void UART1_Putw_Dec(uint32_t w);
void UART1_Putint_Dec(int16_t in);
void UART1_Putintp_Dec(int16_t in);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
void UART1_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz
,int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec);
#endif

//------------------End of File----------------------------

