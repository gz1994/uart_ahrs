#include "ak8975.h"
#include "stm32f10x.h"
#include "IOI2C.h"
#include "eeprom.h"
#include "delay.h"


float AK8975_lastx,AK8975_lasty,AK8975_lastz;
unsigned char AK8975_calib=0;

static int16_t  AK8975_FIFO[3][11]; //磁力计滤波
//磁力计标定值
int16_t  AK8975_maxx=0,AK8975_maxy=0,AK8975_maxz=0,
		 AK8975_minx=-0,AK8975_miny=-0,AK8975_minz=-0;

/**************************实现函数********************************************
*函数原型:	  int16_t my_min(int16_t x,int16_t y)
*功　　能:	   返回两个数值 中最小的那个值
输入参数：    两个 数值
输出参数：   两个值的最小值
*******************************************************************************/
static int16_t my_min(int16_t x,int16_t y)
{
	if(x<y) return x;
		else return y;
}

/**************************实现函数********************************************
*函数原型:	  int16_t my_max(int16_t x,int16_t y)
*功　　能:	   返回两个数值 中最大的那个值
输入参数：    两个 数值
输出参数：   两个值的最小值
*******************************************************************************/
int16_t my_max(int16_t x,int16_t y)
{
	if(x>y) return x;
		else return y;
}

/**************************实现函数********************************************
*函数原型:	   void Ak8975_fifo_init(void)
*功　　能:	   连续读取50次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void Ak8975_fifo_init(void)
{
	int16_t temp[3];
	int i;
	for(i=0;i<50;i++)
	{
		AK8975_getRaw(&temp[0],&temp[1],&temp[2]);
		delay_us(200);  //延时再读取数据
	}
}

/**************************实现函数********************************************
*函数原型:	   void  AK8975_value2fifo(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  AK8975_value2fifo(int16_t x,int16_t y,int16_t z)
{
	unsigned char i , j;
	int32_t sum=0;

	for(i=1;i<10;i++){
		AK8975_FIFO[0][i-1]=AK8975_FIFO[0][i];
		AK8975_FIFO[1][i-1]=AK8975_FIFO[1][i];
		AK8975_FIFO[2][i-1]=AK8975_FIFO[2][i];
	}

	AK8975_FIFO[0][9]=x;
	AK8975_FIFO[1][9]=y;
	AK8975_FIFO[2][9]=z;

	for(j=0;j<3;j++) {	
		sum=0;
		for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
			sum+=AK8975_FIFO[j][i];
		}
		AK8975_FIFO[j][10]=sum/10;	//将平均值更新
	}
} 


/**************************实现函数********************************************
*函数原型:	  void AK8975_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void AK8975_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{
	unsigned char vbuff[7];
	int16_t mx,my,mz;
	
	IICreadBytes(0x18,0x02,7,vbuff);
	
	if(vbuff[0])
	{
		IICwriteByte(0x18,0x0A,0x01);
	}
	
	my = (((int16_t)vbuff[2]) << 8) | vbuff[1];
	mx = (((int16_t)vbuff[4]) << 8) | vbuff[3];
	mz = (((int16_t)vbuff[6]) << 8) | vbuff[5];
	//mz = -mz;
	
	AK8975_value2fifo(mx,my,mz);
	
	*x = AK8975_FIFO[0][10];
	*y = AK8975_FIFO[1][10];
	*z = AK8975_FIFO[2][10];
	
	if(AK8975_calib)
	{
		AK8975_minx = my_min(*x,AK8975_minx); 
		AK8975_maxx = my_max(*x,AK8975_maxx);			
		AK8975_miny = my_min(*y,AK8975_miny); 
		AK8975_maxy = my_max(*y,AK8975_maxy);			
		AK8975_minz = my_min(*z,AK8975_minz); 
		AK8975_maxz = my_max(*z,AK8975_maxz);
	}
			
	*x = *x - ((AK8975_maxx+AK8975_minx)/2);
	*y = *y - ((AK8975_maxy+AK8975_miny)/2);
	*z = *z - ((AK8975_maxz+AK8975_minz)/2);
	
	*z = -(*z);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void AK8975_get_values(int16_t * x, int16_t * y, int16_t * z) 
{
	*x = AK8975_FIFO[0][10];
	*y = AK8975_FIFO[1][10];
	*z = AK8975_FIFO[2][10];
}	

/**************************实现函数********************************************
*函数原型:	  void AK8975_init(void)
*功　　能:	   初始化 AK8975 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void AK8975_init(void)
{ 
	IICwriteByte(0x18,0x0A,0x01);
	delay_ms(10);
	Ak8975_fifo_init();

	Read_Magic_Offset(&AK8975_minx,&AK8975_miny,&AK8975_minz,
		&AK8975_maxx,&AK8975_maxy,&AK8975_maxz); //读取标定值。
}

/**************************实现函数********************************************
*函数原型:	  void AK8975_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：     	
输出参数：  无
*******************************************************************************/
void AK8975_Start_Calib(void)
{
	AK8975_calib = 1;
	AK8975_minx = 0;
	AK8975_maxx = 0;
	AK8975_miny = 0;
	AK8975_maxy = 0;
	AK8975_minz = 0;
	AK8975_maxz = 0;
}

/**************************实现函数********************************************
*函数原型:	  void AK8975_Save_Calib(void)
*功　　能:	  保存磁力计标定值 到Flash
输入参数：     	
输出参数：  无
*******************************************************************************/
void AK8975_Save_Calib(void)
{
	AK8975_calib = 0;
	//将磁力计标定值写入 Flash 保存
	Write_Magic_Offset(AK8975_minx,AK8975_miny,AK8975_minz,
		AK8975_maxx,AK8975_maxy,AK8975_maxz);

	//从 Flash 读取标定数据
	Read_Magic_Offset(&AK8975_minx,&AK8975_miny,&AK8975_minz,
		&AK8975_maxx,&AK8975_maxy,&AK8975_maxz);
}

//------------------End of File----------------------------




