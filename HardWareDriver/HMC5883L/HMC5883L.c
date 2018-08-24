/* HMC5883L.c file


���ܣ�
�ṩHMC5883L ��ʼ�� ��ȡ�����Ƶ�ǰADCת�����
------------------------------------
 */

#include "HMC5883L.h"

//ÿ������ϵ����Ӧ�� LSb/Gauss
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

// x_scale,y_scale,z_scale �¶ȱ仯 ��������
float x_scale,y_scale,z_scale;
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //�������˲�
//�����Ʊ궨ֵ
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,
		 HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;
unsigned char HMC5883_calib=0; //��ʼ����ɱ�־

void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   unsigned char HMC5883_IS_newdata(void)
*��������:	   ��ȡDRDY ���ţ��ж��Ƿ������һ��ת��
 Low for 250 ��sec when data is placed in the data output registers. 
���������  ��
���������  ������ת���������1  ������� 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_FIFO_init(void)
*��������:	   ������ȡ100�����ݣ��Գ�ʼ��FIFO����
���������  ��
���������  ��
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //��ʱ�ٶ�ȡ����
  LED_Change(); //LED��˸
  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
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
	for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

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

	if(HMC5883_calib){//У����Ч�Ļ� �ɼ��궨ֵ
		if(HMC5883_minx>HMC5883_FIFO[0][10])HMC5883_minx=HMC5883_FIFO[0][10];
		if(HMC5883_miny>HMC5883_FIFO[1][10])HMC5883_miny=HMC5883_FIFO[1][10];
		if(HMC5883_minz>HMC5883_FIFO[2][10])HMC5883_minz=HMC5883_FIFO[2][10];

		if(HMC5883_maxx<HMC5883_FIFO[0][10])HMC5883_maxx=HMC5883_FIFO[0][10];
		if(HMC5883_maxy<HMC5883_FIFO[1][10])HMC5883_maxy=HMC5883_FIFO[1][10];
		if(HMC5883_maxz<HMC5883_FIFO[2][10])HMC5883_maxz=HMC5883_FIFO[2][10];
		LED_Change();  //ָʾ ���ڱ궨
	}

} //HMC58X3_newValues

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  IICwriteByte(HMC58X3_ADDR,reg,val);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
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
	
	//ת����
	/*temp = mag_value[0];
	mag_value[0] = -mag_value[1];
	mag_value[1] = mag_value[2];
	mag_value[2] = -temp;*/
	
	HMC58X3_newValues(mag_value[0],mag_value[1],mag_value[2]);
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*��������:	   ��ȡ �����Ƶĵ�ǰADCֵ
���������    �������Ӧ�����ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5883_FIFO[0][10];
  *y = HMC5883_FIFO[1][10]; 
  *z = HMC5883_FIFO[2][10]; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5883_lastx=(float)(xr-((HMC5883_maxx+HMC5883_minx)/2));
  arry[1]= HMC5883_lasty=(float)(yr-((HMC5883_maxy+HMC5883_miny)/2));
  arry[2]= HMC5883_lastz=(float)(zr-((HMC5883_maxz+HMC5883_minz)/2));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setGain(unsigned char gain)
*��������:	   ���� 5883L������
���������     Ŀ������ 0-7
���������  ��
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setMode(unsigned char mode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  delay_us(100);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_init(u8 setmode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  int16_t my_min(int16_t x,int16_t y)
*��������:	   ����������ֵ ����С���Ǹ�ֵ
���������    ���� ��ֵ
���������   ����ֵ����Сֵ
*******************************************************************************/
int16_t my_min(int16_t x,int16_t y)
{
	if(x<y) return x;
		else return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_calibrate(unsigned char gain,unsigned int n_samples)
*��������:	   ʹ��HMC5883�ڲ��Ĵų��������ı궨
���������    gain  ���� 0-7
			  n_samples �������� 
���������  ��
*******************************************************************************/
void HMC58X3_calibrate(unsigned char gain,unsigned int n_samples) 
{
    int16_t xyz[3];               // 16 bit integer values for each axis.
    int32_t xyz_total[3]={0,0,0}; // 32 bit totals so they won't overflow.
    unsigned char bret=1;         // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    char id[3];                   // Three identification registers should return 'H43'.
    int32_t low_limit, high_limit;  
	unsigned int i;    
	unsigned char vbuff[6]; //��ȡHMC5883 ���ֽ����ݻ���                             

    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        HMC58X3_getID(id); //��ȡоƬID������Ƿ��Ǵ�����оƬ��
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


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setDOR(unsigned char DOR)
*��������:	   ���� 5883L�� �����������
���������     ����ֵ
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
���������  ��
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getID(char id[3])
*��������:	   ��ȡоƬ��ID
���������     	ID��ŵ�����
���������  ��
*******************************************************************************/
void HMC58X3_getID(char id[3]) 
{
      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
}   // getID().

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_SetUp(void)
*��������:	   ��ʼ�� HMC5883L ʹ֮�������״̬
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
  HMC5883_calib=0;
  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  // Single mode conversion was used in calibration, now set continuous mode
  HMC58X3_calibrate(1, 32); //�Լ�� �궨
  HMC58X3_setMode(0);
  HMC58X3_setDOR(6);  //75hz ������
  HMC58X3_FIFO_init();

  Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
  &HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz); //��ȡ�궨ֵ��
  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Start_Calib(void)
*��������:	   ��������Ʊ궨
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib=1;//��ʼ�궨
	HMC5883_maxx=0;	//��ԭ���ı궨ֵ���
	HMC5883_maxy=0;
	HMC5883_maxz=0;
	HMC5883_minx=-0;
	HMC5883_miny=-0;
	HMC5883_minz=-0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Save_Calib(void)
*��������:	  ��������Ʊ궨ֵ ��Flash
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_Save_Calib(void){
	//�������Ʊ궨ֵд�� Flash ����
	Write_Magic_Offset(HMC5883_minx,HMC5883_miny,HMC5883_minz,
	HMC5883_maxx,HMC5883_maxy,HMC5883_maxz);

	HMC5883_calib=0; //�����궨
	//�� Flash ��ȡ�궨����
	Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
  	&HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz);
}	//HMC5883L_Save_Calib()

//------------------End of File----------------------------
