#ifndef __AK8975_H__
#define __AK8975_H__
#include "stm32f10x.h"

extern unsigned char AK8975_calib;

extern int16_t  AK8975_maxx,AK8975_maxy,AK8975_maxz,
		 AK8975_minx,AK8975_miny,AK8975_minz;

void AK8975_init(void);
void AK8975_getRaw(int16_t *x,int16_t *y,int16_t *z);
void AK8975_get_values(int16_t * x, int16_t * y, int16_t * z);
void AK8975_Start_Calib(void);
void AK8975_Save_Calib(void);


#endif /* __AK8975_H__*/





