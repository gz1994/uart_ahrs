/* main.c file

姿态解算 IMU
将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
------------------------------------
 */

#include "IMU.h"

struct AHRS_PARAM ahrs1,ahrs2;

void MadgwickAHRSupdate(struct AHRS_PARAM ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer3(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	/* TIM2 configuration*/ 
  /* Time Base configuration 基本配置 配置定时器的时基单元*/
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
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
	/* 定时器配置:
	1.设置定时器最大计数值 50000
	2.设置时钟分频系数：TIM_CKD_DIV1
	3. 设置预分频：  1Mhz/50000= 1hz 
	4.定时器计数模式  向上计数模式
	*/		 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
  	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	/* Enable the TIM3 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  	//启动定时器
	TIM_Cmd(TIM3, ENABLE); 
  	TIM_Cmd(TIM2, ENABLE);                  
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
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

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM3->CNT; //读低16位时间
 	return temp;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
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
  	ahrs1.q0 = 1.0f;  //初始化四元数
  	ahrs1.q1 = 0.0f;
  	ahrs1.q2 = 0.0f;
  	ahrs1.q3 = 0.0f;
  	ahrs1.exInt = 0.0;
  	ahrs1.eyInt = 0.0;
  	ahrs1.ezInt = 0.0;
  	ahrs1.lastUpdate = micros();//更新时间
  	ahrs1.now = micros();
	
	// initialize quaternion
  	ahrs2.q0 = 1.0f;  //初始化四元数
  	ahrs2.q1 = 0.0f;
  	ahrs2.q2 = 0.0f;
  	ahrs2.q3 = 0.0f;
  	ahrs2.exInt = 0.0;
  	ahrs2.eyInt = 0.0;
  	ahrs2.ezInt = 0.0;
  	ahrs2.lastUpdate = micros();//更新时间
  	ahrs2.now = micros();
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];

	//读取加速度和陀螺仪的当前ADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	
	values[0] = (float) accgyroval[0];
	values[1] = (float) accgyroval[1];
	values[2] = (float) accgyroval[2];
	
	values[3] = ((float) accgyroval[3]) / 32.8;//16.4f; //转成度每秒
	values[4] = ((float) accgyroval[4]) / 32.8;//16.4f; //转成度每秒
	values[5] = ((float) accgyroval[5]) / 32.8;//16.4f; //转成度每秒
		//这里已经将量程改成了 2000度每秒  32.8 对应 1度每秒
		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒

    HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
}

/**************************实现函数********************************************
*函数原型:	   void IMU_update
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
float Kp = 10.0f;   // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.005f;   // integral gain governs rate of convergence of gyroscope biases

void IMU_update(struct AHRS_PARAM * ahrs, float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez,halfT;

	// 先把这些用得到的值算好
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

	ahrs->now = micros();  //读取时间
	if(ahrs->now < ahrs->lastUpdate){ //定时器溢出过了。
		halfT =  ((float)(ahrs->now + (0xffff- ahrs->lastUpdate)) / 2000000.0f);
	}
	else	{
		halfT =  ((float)(ahrs->now - ahrs->lastUpdate) / 2000000.0f);
	}
	ahrs->lastUpdate = ahrs->now;	//更新时间

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

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
float Kp2 = 5.0f;   // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki2 = 0.005f;   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(struct AHRS_PARAM * ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez,halfT;

	// 先把这些用得到的值算好
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

	ahrs->now = micros();  //读取时间
	if(ahrs->now < ahrs->lastUpdate){ //定时器溢出过了。
		halfT =  ((float)(ahrs->now + (0xffff- ahrs->lastUpdate)) / 2000000.0f);
	}
	else	{
		halfT =  ((float)(ahrs->now - ahrs->lastUpdate) / 2000000.0f);
	}
	ahrs->lastUpdate = ahrs->now;	//更新时间

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


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	float q[4]; //　四元数
	float mygetqval[9];	//用于存放传感器转换结果的数组
	
	IMU_getValues(mygetqval);	 

	//加速度和磁力计保持 ADC值　不需要转换
	IMU_update(&ahrs1, mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180, mygetqval[0], mygetqval[1], mygetqval[2]);
	
	//IMU_AHRSupdate(&ahrs2, mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
	//	mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

	q[0] = ahrs1.q0; //返回当前值
	q[1] = ahrs1.q1;
	q[2] = ahrs1.q2;
	q[3] = ahrs1.q3;
	angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
	angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
	/*q[0] = ahrs2.q0; //返回当前值
	q[1] = ahrs2.q1;
	q[2] = ahrs2.q2;
	q[3] = ahrs2.q3;
	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
	*/
	//angles[0] = 0;

	//if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}

//------------------End of File----------------------------
