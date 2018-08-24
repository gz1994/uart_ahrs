/* HMC5883L.c file


���ܣ�
�ṩHMC5883L ��ʼ�� ��ȡ�����Ƶ�ǰADCת�����
------------------------------------
 */

#include "HMC5883L.h"

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

	HMC5883_FIFO[0][9]=(float)x;
	HMC5883_FIFO[1][9]=(float)y;
	HMC5883_FIFO[2][9]=(float)z;

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
	}

} //HMC58X3_newValues


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
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
*����ԭ��:	  int16_t my_max(int16_t x,int16_t y)
*��������:	   ����������ֵ �������Ǹ�ֵ
���������    ���� ��ֵ
���������   ����ֵ����Сֵ
*******************************************************************************/
int16_t my_max(int16_t x,int16_t y)
{
	if(x>y) return x;
		else return y;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_SetUp(void)
*��������:	   ��ʼ�� HMC5883L ʹ֮�������״̬
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
	HMC5883_calib=0;

	HMC58X3_FIFO_init();

	Read_Magic_Offset(&HMC5883_minx,&HMC5883_miny,&HMC5883_minz,
	&HMC5883_maxx,&HMC5883_maxy,&HMC5883_maxz); //��ȡ�궨ֵ��

	IICwriteByte(0x18,0x0A,0x01);
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
