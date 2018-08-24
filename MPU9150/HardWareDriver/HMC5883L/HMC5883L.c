/* HMC5883L.c file


功能：
提供HMC5883L 初始化 读取磁力计当前ADC转换结果
------------------------------------
 */

#include "HMC5883L.h"

float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //磁力计滤波
//磁力计标定值
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,
		 HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;
unsigned char HMC5883_calib=0; //初始化完成标志

void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************实现函数********************************************
*函数原型:	   unsigned char HMC5883_IS_newdata(void)
*功　　能:	   读取DRDY 引脚，判断是否完成了一次转换
 Low for 250 μsec when data is placed in the data output registers. 
输入参数：  无
输出参数：  如果完成转换，则输出1  否则输出 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
	u8 res;
 	res = I2C_ReadOneByte(0x18, 0x02);
	if(res)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
	  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
	  delay_us(200);  //延时再读取数据
  }
}

/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=(float)x;
	HMC5883_FIFO[1][9]=(float)y;
	HMC5883_FIFO[2][9]=(float)z;

	sum=0;
	for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//将平均值更新

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;

	if(HMC5883_calib){//校正有效的话 采集标定值
		if(HMC5883_minx>HMC5883_FIFO[0][10])HMC5883_minx=HMC5883_FIFO[0][10];
		if(HMC5883_miny>HMC5883_FIFO[1][10])HMC5883_miny=HMC5883_FIFO[1][10];
		if(HMC5883_minz>HMC5883_FIFO[2][10])HMC5883_minz=HMC5883_FIFO[2][10];

		if(HMC5883_maxx<HMC5883_FIFO[0][10])HMC5883_maxx=HMC5883_FIFO[0][10];
		if(HMC5883_maxy<HMC5883_FIFO[1][10])HMC5883_maxy=HMC5883_FIFO[1][10];
		if(HMC5883_maxz<HMC5883_FIFO[2][10])HMC5883_maxz=HMC5883_FIFO[2][10];
	}

} //HMC58X3_newValues


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{
	unsigned char vbuff[7];
	int16_t mx,my,mz;
	vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
	
	IICreadBytes(0x18,0x02,7,vbuff);
	
	if(vbuff[0])
	{
		IICwriteByte(0x18,0x0A,0x01);
	}
	
	my = (((int16_t)vbuff[2]) << 8) | vbuff[1];
	mx = (((int16_t)vbuff[4]) << 8) | vbuff[3];
	mz = (((int16_t)vbuff[6]) << 8) | vbuff[5];
	//mz = -mz;
	
	HMC58X3_newValues(mx,my,mz);
	
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5883_FIFO[0][10];
  *y = HMC5883_FIFO[1][10]; 
  *z = HMC5883_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5883_lastx=(float)(xr-((HMC5883_maxx+HMC5883_minx)/2));
  arry[1]= HMC5883_lasty=(float)(yr-((HMC5883_maxy+HMC5883_miny)/2));
  arry[2]= HMC5883_lastz=(float)(zr-((HMC5883_maxz+HMC5883_minz)/2));
}


/**************************实现函数********************************************
*函数原型:	  int16_t my_min(int16_t x,int16_t y)
*功　　能:	   返回两个数值 中最小的那个值
输入参数：    两个 数值
输出参数：   两个值的最小值
*******************************************************************************/
int16_t my_min(int16_t x,int16_t y)
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
*函数原型:	  void HMC5883L_SetUp(void)
*功　　能:	   初始化 HMC5883L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
	HMC5883_calib=0;

	HMC58X3_FIFO_init();

	Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
	&HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz); //读取标定值。

	IICwriteByte(0x18,0x0A,0x01);
}

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib=1;//开始标定
	HMC5883_maxx=0;	//将原来的标定值清除
	HMC5883_maxy=0;
	HMC5883_maxz=0;
	HMC5883_minx=-0;
	HMC5883_miny=-0;
	HMC5883_minz=-0;
}

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Save_Calib(void)
*功　　能:	  保存磁力计标定值 到Flash
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_Save_Calib(void){
	//将磁力计标定值写入 Flash 保存
	Write_Magic_Offset(HMC5883_minx,HMC5883_miny,HMC5883_minz,
	HMC5883_maxx,HMC5883_maxy,HMC5883_maxz);

	HMC5883_calib=0; //结束标定
	//从 Flash 读取标定数据
	Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
  	&HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz);
}	//HMC5883L_Save_Calib()

//------------------End of File----------------------------
