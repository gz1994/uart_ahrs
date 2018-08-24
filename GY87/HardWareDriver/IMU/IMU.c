/* main.c file

��̬���� IMU
�������������ֵ������̬���㡣�õ�Ŀ������ĸ����Ǻͺ���� �ͺ����
------------------------------------
 */

#include "IMU.h"

struct AHRS_PARAM ahrs1,ahrs2;

void MadgwickAHRSupdate(struct AHRS_PARAM ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_Timer3(void)
*��������:	  ��ʼ��Tim2  Tim3 ��������ʱ���������Բ���һ��32λ�Ķ�ʱ�����ṩϵͳus ���ļ�ʱ	
�����������
���������û��	
*******************************************************************************/
void Initial_Timer3(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration �������� ���ö�ʱ����ʱ����Ԫ*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //�Զ���װֵ         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
  TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
  /* Disable the TIM2 Update event */
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  /* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
  /* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
  /* Use the External Clock as TIM2 Slave Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
  /* Enable the TIM2 Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);	
	/* ��ʱ������:
	1.���ö�ʱ��������ֵ 50000
	2.����ʱ�ӷ�Ƶϵ����TIM_CKD_DIV1
	3. ����Ԥ��Ƶ��  1Mhz/50000= 1hz 
	4.��ʱ������ģʽ  ���ϼ���ģʽ
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M ��ʱ��  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Ӧ�����õ�TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// ʹ��TIM3���ؼĴ���ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
  	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	/* Enable the TIM3 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  	//������ʱ��
	TIM_Cmd(TIM3, ENABLE); 
  	TIM_Cmd(TIM2, ENABLE);                  
}

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint32_t micros(void)
*��������:	  ��ȡϵͳ���е�ʱ�� �����ص�λΪus ��ʱ������	
�����������
�����������������ǰʱ�䣬���ϵ翪ʼ��ʱ  ��λ us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //����16λʱ��
 	temp = temp<<16;
 	temp += TIM3->CNT; //����16λʱ��
 	return temp;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_init(void)
*��������:	  ��ʼ��IMU���	
			  ��ʼ������������
			  ��ʼ����Ԫ��
			  ����������
			  ����ϵͳʱ��
�����������
���������û��
*******************************************************************************/
void IMU_init(void)
{	 
	MPU6050_initialize();
	HMC5883L_SetUp();
	delay_ms(50);
	MPU6050_initialize();
	HMC5883L_SetUp();
	Initial_Timer3();
	// initialize quaternion
  	ahrs1.q0 = 1.0f;  //��ʼ����Ԫ��
  	ahrs1.q1 = 0.0f;
  	ahrs1.q2 = 0.0f;
  	ahrs1.q3 = 0.0f;
  	ahrs1.exInt = 0.0;
  	ahrs1.eyInt = 0.0;
  	ahrs1.ezInt = 0.0;
  	ahrs1.lastUpdate = micros();//����ʱ��
  	ahrs1.now = micros();
	
	// initialize quaternion
  	ahrs2.q0 = 1.0f;  //��ʼ����Ԫ��
  	ahrs2.q1 = 0.0f;
  	ahrs2.q2 = 0.0f;
  	ahrs2.q3 = 0.0f;
  	ahrs2.exInt = 0.0;
  	ahrs2.eyInt = 0.0;
  	ahrs2.ezInt = 0.0;
  	ahrs2.lastUpdate = micros();//����ʱ��
  	ahrs2.now = micros();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���������û��
*******************************************************************************/
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];

	//��ȡ���ٶȺ������ǵĵ�ǰADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	
	values[0] = (float) accgyroval[0];
	values[1] = (float) accgyroval[1];
	values[2] = (float) accgyroval[2];
	
	values[3] = ((float) accgyroval[3]) / 32.8;//16.4f; //ת�ɶ�ÿ��
	values[4] = ((float) accgyroval[4]) / 32.8;//16.4f; //ת�ɶ�ÿ��
	values[5] = ((float) accgyroval[5]) / 32.8;//16.4f; //ת�ɶ�ÿ��
		//�����Ѿ������̸ĳ��� 2000��ÿ��  32.8 ��Ӧ 1��ÿ��
		//�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��

    HMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_update
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
float Kp = 10.0f;   // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.005f;   // integral gain governs rate of convergence of gyroscope biases

void IMU_update(struct AHRS_PARAM * ahrs, float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez,halfT;

	// �Ȱ���Щ�õõ���ֵ���
	float q0q0 = ahrs->q0*ahrs->q0;
	float q0q1 = ahrs->q0*ahrs->q1;
	float q0q2 = ahrs->q0*ahrs->q2;
	float q0q3 = ahrs->q0*ahrs->q3;
	float q1q1 = ahrs->q1*ahrs->q1;
	float q1q2 = ahrs->q1*ahrs->q2;
	float q1q3 = ahrs->q1*ahrs->q3;
	float q2q2 = ahrs->q2*ahrs->q2;   
	float q2q3 = ahrs->q2*ahrs->q3;
	float q3q3 = ahrs->q3*ahrs->q3;          

	ahrs->now = micros();  //��ȡʱ��
	if(ahrs->now < ahrs->lastUpdate){ //��ʱ��������ˡ�
		halfT =  ((float)(ahrs->now + (0xffff- ahrs->lastUpdate)) / 2000000.0f);
	}
	else	{
		halfT =  ((float)(ahrs->now - ahrs->lastUpdate) / 2000000.0f);
	}
	ahrs->lastUpdate = ahrs->now;	//����ʱ��

	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;   

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
		ahrs->exInt = ahrs->exInt + ex * Ki * halfT;
		ahrs->eyInt = ahrs->eyInt + ey * Ki * halfT;	
		ahrs->ezInt = ahrs->ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements
		gx = gx + Kp*ex + ahrs->exInt;
		gy = gy + Kp*ey + ahrs->eyInt;
		gz = gz + Kp*ez + ahrs->ezInt;
	}

	// integrate quaternion rate and normalise
	ahrs->q0 = ahrs->q0 + (-ahrs->q1*gx - ahrs->q2*gy - ahrs->q3*gz)*halfT;
	ahrs->q1 = ahrs->q1 + (ahrs->q0*gx + ahrs->q2*gz - ahrs->q3*gy)*halfT;
	ahrs->q2 = ahrs->q2 + (ahrs->q0*gy - ahrs->q1*gz + ahrs->q3*gx)*halfT;
	ahrs->q3 = ahrs->q3 + (ahrs->q0*gz + ahrs->q1*gy - ahrs->q2*gx)*halfT;  

	// normalise quaternion
	norm = invSqrt(ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
	ahrs->q0 = ahrs->q0 * norm;
	ahrs->q1 = ahrs->q1 * norm;
	ahrs->q2 = ahrs->q2 * norm;
	ahrs->q3 = ahrs->q3 * norm;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
float Kp2 = 5.0f;   // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki2 = 0.005f;   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(struct AHRS_PARAM * ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez,halfT;

	// �Ȱ���Щ�õõ���ֵ���
	float q0q0 = ahrs->q0*ahrs->q0;
	float q0q1 = ahrs->q0*ahrs->q1;
	float q0q2 = ahrs->q0*ahrs->q2;
	float q0q3 = ahrs->q0*ahrs->q3;
	float q1q1 = ahrs->q1*ahrs->q1;
	float q1q2 = ahrs->q1*ahrs->q2;
	float q1q3 = ahrs->q1*ahrs->q3;
	float q2q2 = ahrs->q2*ahrs->q2;   
	float q2q3 = ahrs->q2*ahrs->q3;
	float q3q3 = ahrs->q3*ahrs->q3;          

	ahrs->now = micros();  //��ȡʱ��
	if(ahrs->now < ahrs->lastUpdate){ //��ʱ��������ˡ�
		halfT =  ((float)(ahrs->now + (0xffff- ahrs->lastUpdate)) / 2000000.0f);
	}
	else	{
		halfT =  ((float)(ahrs->now - ahrs->lastUpdate) / 2000000.0f);
	}
	ahrs->lastUpdate = ahrs->now;	//����ʱ��

	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = invSqrt(mx*mx + my*my + mz*mz);          
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	// compute reference direction of flux
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;     

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
		ahrs->exInt = ahrs->exInt + ex * Ki2 * halfT;
		ahrs->eyInt = ahrs->eyInt + ey * Ki2 * halfT;	
		ahrs->ezInt = ahrs->ezInt + ez * Ki2 * halfT;

		// adjusted gyroscope measurements
		gx = gx + Kp2*ex + ahrs->exInt;
		gy = gy + Kp2*ey + ahrs->eyInt;
		gz = gz + Kp2*ez + ahrs->ezInt;
	}

	// integrate quaternion rate and normalise
	ahrs->q0 = ahrs->q0 + (-ahrs->q1*gx - ahrs->q2*gy - ahrs->q3*gz)*halfT;
	ahrs->q1 = ahrs->q1 + (ahrs->q0*gx + ahrs->q2*gz - ahrs->q3*gy)*halfT;
	ahrs->q2 = ahrs->q2 + (ahrs->q0*gy - ahrs->q1*gz + ahrs->q3*gx)*halfT;
	ahrs->q3 = ahrs->q3 + (ahrs->q0*gz + ahrs->q1*gy - ahrs->q2*gx)*halfT;  

	// normalise quaternion
	norm = invSqrt(ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
	ahrs->q0 = ahrs->q0 * norm;
	ahrs->q1 = ahrs->q1 * norm;
	ahrs->q2 = ahrs->q2 * norm;
	ahrs->q3 = ahrs->q3 * norm;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	float q[4]; //����Ԫ��
	float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
	
	IMU_getValues(mygetqval);	 

	//���ٶȺʹ����Ʊ��� ADCֵ������Ҫת��
	IMU_update(&ahrs1, mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180, mygetqval[0], mygetqval[1], mygetqval[2]);
	
	//IMU_AHRSupdate(&ahrs2, mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
	//	mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

	q[0] = ahrs1.q0; //���ص�ǰֵ
	q[1] = ahrs1.q1;
	q[2] = ahrs1.q2;
	q[3] = ahrs1.q3;
	angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
	angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
	/*q[0] = ahrs2.q0; //���ص�ǰֵ
	q[1] = ahrs2.q1;
	q[2] = ahrs2.q2;
	q[3] = ahrs2.q3;
	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
	*/
	//angles[0] = 0;

	//if(angles[0]<0)angles[0]+=360.0f;  //�� -+180��  ת��0-360��
}

//------------------End of File----------------------------
