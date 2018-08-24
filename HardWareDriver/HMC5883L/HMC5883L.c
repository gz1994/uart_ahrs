/* HMC5883L.c file


功能：
提供HMC5883L 初始化 读取磁力计当前ADC转换结果
------------------------------------
 */

#include "HMC5883L.h"

//每个增益系数对应的 LSb/Gauss
const int16_t counts_per_milligauss[8]={  
    1370,  //+-0.88Ga
    1090,  //+-1.3Ga
    820,   //+-1.9Ga
    660,   //+-2.5Ga
    440,   //+-4.0Ga
    390,   //+-4.7Ga
    330,   //+-5.6Ga
    230	   //+-8.1Ga
  };

// x_scale,y_scale,z_scale 温度变化 比例因子
float x_scale,y_scale,z_scale;
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
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
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
  LED_Change(); //LED闪烁
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

	HMC5883_FIFO[0][9]=(((float)x)/x_scale);
	HMC5883_FIFO[1][9]=(((float)y)/y_scale);
	HMC5883_FIFO[2][9]=(((float)z)/z_scale);

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
		LED_Change();  //指示 正在标定
	}

} //HMC58X3_newValues

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  IICwriteByte(HMC58X3_ADDR,reg,val);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
	unsigned char vbuff[6];
	int16_t mag_value[3];
	int16_t temp;
	vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
	IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
	mag_value[0] = ((int16_t)vbuff[0] << 8) | vbuff[1];
	mag_value[1] = ((int16_t)vbuff[4] << 8) | vbuff[5];
	mag_value[2] = ((int16_t)vbuff[2] << 8) | vbuff[3];
	
	//转换轴
	/*temp = mag_value[0];
	mag_value[0] = -mag_value[1];
	mag_value[1] = mag_value[2];
	mag_value[2] = -temp;*/
	
	HMC58X3_newValues(mag_value[0],mag_value[1],mag_value[2]);
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
*函数原型:	  void HMC58X3_setGain(unsigned char gain)
*功　　能:	   设置 5883L的增益
输入参数：     目标增益 0-7
输出参数：  无
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setMode(unsigned char mode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  delay_us(100);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_init(u8 setmode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }
  x_scale=1.0; // get actual values
  y_scale=1.0;
  z_scale=1.0;

  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
  HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

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
*函数原型:	  void HMC58X3_calibrate(unsigned char gain,unsigned int n_samples)
*功　　能:	   使用HMC5883内部的磁场做初步的标定
输入参数：    gain  增益 0-7
			  n_samples 采样次数 
输出参数：  无
*******************************************************************************/
void HMC58X3_calibrate(unsigned char gain,unsigned int n_samples) 
{
    int16_t xyz[3];               // 16 bit integer values for each axis.
    int32_t xyz_total[3]={0,0,0}; // 32 bit totals so they won't overflow.
    unsigned char bret=1;         // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    char id[3];                   // Three identification registers should return 'H43'.
    int32_t low_limit, high_limit;  
	unsigned int i;    
	unsigned char vbuff[6]; //读取HMC5883 的字节数据缓冲                             

    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        HMC58X3_getID(id); //读取芯片ID，检查是否是磁力计芯片。
        if (('H' == id[0]) && ('4' == id[1]) && ('3' == id[2]))
        {   /*
                Use the positive bias current to impose a known field on each axis.
                This field depends on the device and the axis.
            */
            HMC58X3_writeReg(HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
            /*
                Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
                The new gain setting is effective from the second measurement and on.
            */
            HMC58X3_setGain(gain);                      
            HMC58X3_setMode(1);                         // Change to single measurement mode.
            // Get the raw values and ignore since this reading may use previous gain.
			IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
			xyz[0]=((int16_t)vbuff[0] << 8) | vbuff[1];
			xyz[1]=((int16_t)vbuff[4] << 8) | vbuff[5];
			xyz[2]=((int16_t)vbuff[2] << 8) | vbuff[3];
			 
            for ( i=0; i<n_samples; i++) 
            { 
                HMC58X3_setMode(1);
                IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
				xyz[0]=((int16_t)vbuff[0] << 8) | vbuff[1];
				xyz[1]=((int16_t)vbuff[4] << 8) | vbuff[5];
				xyz[2]=((int16_t)vbuff[2] << 8) | vbuff[3];
				/*
                    Since the measurements are noisy, they should be averaged rather than taking the max.
                */
                xyz_total[0]+=xyz[0];
                xyz_total[1]+=xyz[1];
                xyz_total[2]+=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= my_min(xyz[0],my_min(xyz[1],xyz[2])))
                {
                    //HMC58x3 Self test saturated. Increase range.
                    bret=0;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Apply the negative bias. (Same gain)
            */
            HMC58X3_writeReg(HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
            for (i=0; i<n_samples; i++) 
            { 
                HMC58X3_setMode(1);
                // Get the raw values and ignore since this reading may use previous gain.
				IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
				xyz[0]=((int16_t)vbuff[0] << 8) | vbuff[1];
				xyz[1]=((int16_t)vbuff[4] << 8) | vbuff[5];
				xyz[2]=((int16_t)vbuff[2] << 8) | vbuff[3];
				/*
                    Since the measurements are noisy, they should be averaged.
                */
                xyz_total[0]-=xyz[0];
                xyz_total[1]-=xyz[1];
                xyz_total[2]-=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= my_min(xyz[0],my_min(xyz[1],xyz[2])))
                {
                    //HMC58x3 Self test saturated. Increase range.
                    bret=0;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Compare the values against the expected self test bias gauss.
                Notice, the same limits are applied to all axis.
            */
            low_limit =SELF_TEST_LOW_LIMIT *counts_per_milligauss[gain]*2*n_samples;
            high_limit=SELF_TEST_HIGH_LIMIT*counts_per_milligauss[gain]*2*n_samples;

            if ((bret) && 
                (low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
                (low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
                (low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2]) )
            {   /*
                    Successful calibration.
                    Normalize the scale factors so all axis return the same range of values for the bias field.
                    Factor of 2 is from summation of total of n_samples from both positive and negative bias.
                */
                x_scale=(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
                y_scale=(counts_per_milligauss[gain]*(HMC58X3_Y_SELF_TEST_GAUSS*2))/(xyz_total[1]/n_samples);
                z_scale=(counts_per_milligauss[gain]*(HMC58X3_Z_SELF_TEST_GAUSS*2))/(xyz_total[2]/n_samples);
            }else
            {
                //HMC58x3 Self test out of range.
                bret=0;
            }
            HMC58X3_writeReg(HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default.
        }else
        {
            //HMC5843 failed id check.
            bret=0;
        }
    }else
    {  
        //HMC58x3 Bad parameters.
        bret=0;
    }
	HMC58X3_writeReg(HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default
}   //  HMC58X3_calibrate().


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setDOR(unsigned char DOR)
*功　　能:	   设置 5883L的 数据输出速率
输入参数：     速率值
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
输出参数：  无
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
*******************************************************************************/
void HMC58X3_getID(char id[3]) 
{
      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
}   // getID().

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_SetUp(void)
*功　　能:	   初始化 HMC5883L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
  HMC5883_calib=0;
  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  // Single mode conversion was used in calibration, now set continuous mode
  HMC58X3_calibrate(1, 32); //自检测 标定
  HMC58X3_setMode(0);
  HMC58X3_setDOR(6);  //75hz 更新率
  HMC58X3_FIFO_init();

  Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
  &HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz); //读取标定值。
  
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
