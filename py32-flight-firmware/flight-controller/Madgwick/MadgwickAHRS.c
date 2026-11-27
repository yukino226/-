//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include "commom.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define AHRS_DELAY_FIX     1


#if (AHRS_DELAY_FIX != 1)
//---------------------------------------------------------------------------------------------------
// Variable definitions
static float beta = 1.0f;					// 2 * proportional gain (Kp)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float T13, T23, T33, T11, T12;
float T21, T22, T31, T32;
float HMC_DECLINATION,HMC_DECLINATION_R;
volatile float Theta_R, Gamma_R, Phi_R;
volatile float Theta,   Gamma,   Phi;
//---------------------------------------------------------------------------------------------------
// Function declarations
//====================================================================================================
// Functions
//---------------------------------------------------------------------------------------------------
void MadgwickAHRInit(void)
{
	
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

			s0 = constrain_float(s0, -0.1f, 0.1f);
			s1 = constrain_float(s1, -0.1f, 0.1f);
			s2 = constrain_float(s2, -0.1f, 0.1f);
			s3 = constrain_float(s3, -0.1f, 0.1f);

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f * dt);
	q1 += qDot2 * (1.0f * dt);
	q2 += qDot3 * (1.0f * dt);
	q3 += qDot4 * (1.0f * dt);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	///*
	T13 = 2 * (q1 * q3 - q0 * q2);
	if (fabsf(T13) > 1) 
        {
            if (T13 > 0) 
            {
                T13 = 1;
            }
            else 
            {
                T13 = -1;
            }
	}

	Theta_R = -asinf(T13);
	Theta = Theta_R * RAD2DEG;

	T23 = 2 * (q2 * q3 + q0 * q1);
	T33 = q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0;

	Gamma_R = atan2f(T23, T33);
	Gamma = Gamma_R * RAD2DEG;

	T12 = 2 * (q1 * q2 + q0 * q3);
	T11 = -q2 * q2 - q3 * q3 + q1 * q1 + q0 * q0;

	Phi_R = atan2f(T12, T11);
        
	Phi_R += HMC_DECLINATION_R;   // 磁偏角处理
	if (Phi_R > PI) 
        {
	    Phi_R -= PI * 2;
	}
	else if (Phi_R < -PI) 
        {
	    Phi_R += PI * 2;
	}
        
	Phi = Phi_R * RAD2DEG;

	// 计算矩阵的其它元素
	T21 = 2 * (q1 * q2 - q0 * q3);
	T22 = q2 * q2 - q3 * q3 + q0 * q0 - q1 * q1;
	T31 = 2 * (q1 * q3 + q0 * q2);
	T32 = 2 * (q2 * q3 - q0 * q1);
}

void MadgwickAHR_Set_Beat(float p)
{
	beta = p;
}

void MadgwickAHR_ClearInt(void)
{

}
#else

#define Quad_Num  10
float    Quad_Buf[Quad_Num][4]={0};
float    Acc[Quad_Num][3] = {0};
uint16_t Quad_Delay=5;
float beta = 1.0f;					// 2 * proportional gain (Kp)
#define betaDef		20.0f
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float T13, T23, T33, T11, T12;
float T21, T22, T31, T32;
volatile float Theta_R, Gamma_R, Phi_R;
volatile float Theta,   Gamma,   Phi;
#define AHRS_KI     0.01f
#define AHRS_KP     0.000f
float exInt=0,eyInt=0,ezInt=0; 
void MadgwickAHRInit(void)
{

}

void MadgwickAHR_ClearInt(void)
{
	exInt = 0;
	eyInt = 0;
	ezInt = 0;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float delta;

	static float vx=0,vy=0,vz=0;
	static float ex=0,ey=0,ez=0; 
	
	static uint16_t sync_cnt=0;
	sync_cnt++;
	if(sync_cnt>=10)
	{
		for(uint8_t i=Quad_Num-1;i>0;i--)//将四元数历史值保存起来
		{
			Quad_Buf[i][0]=Quad_Buf[i-1][0];
			Quad_Buf[i][1]=Quad_Buf[i-1][1];
			Quad_Buf[i][2]=Quad_Buf[i-1][2];
			Quad_Buf[i][3]=Quad_Buf[i-1][3];
			Acc[i][0] = Acc[i-1][0];
			Acc[i][1] = Acc[i-1][1];
			Acc[i][2] = Acc[i-1][2];
		}
	
		Quad_Buf[0][0]=q0;
		Quad_Buf[0][1]=q1;
		Quad_Buf[0][2]=q2;
		Quad_Buf[0][3]=q3;
	
		Acc[0][0] = ax;
		Acc[0][1] = ay;
		Acc[0][2] = az;

		sync_cnt = 0;
	}	


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		ax=Acc[5][0];
		ay=Acc[5][1];
		az=Acc[5][2];

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * Quad_Buf[Quad_Delay][0];
		_2q1 = 2.0f * Quad_Buf[Quad_Delay][1];
		_2q2 = 2.0f * Quad_Buf[Quad_Delay][2];
		_2q3 = 2.0f * Quad_Buf[Quad_Delay][3];
		_4q0 = 4.0f * Quad_Buf[Quad_Delay][0];
		_4q1 = 4.0f * Quad_Buf[Quad_Delay][1];
		_4q2 = 4.0f * Quad_Buf[Quad_Delay][2];
		_8q1 = 8.0f * Quad_Buf[Quad_Delay][1];
		_8q2 = 8.0f * Quad_Buf[Quad_Delay][2];
		q0q0 = Quad_Buf[5][0] * Quad_Buf[Quad_Delay][0];
		q1q1 = Quad_Buf[5][1] * Quad_Buf[Quad_Delay][1];
		q2q2 = Quad_Buf[5][2] * Quad_Buf[Quad_Delay][2];
		q3q3 = Quad_Buf[5][3] * Quad_Buf[Quad_Delay][3];

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 *  Quad_Buf[Quad_Delay][1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 *  Quad_Buf[Quad_Delay][2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 *  Quad_Buf[Quad_Delay][3] - _2q1 * ax + 4.0f * q2q2 *  Quad_Buf[Quad_Delay][3] - _2q2 * ay;
		
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;


		// Apply feedback step	
		// Rate of change of quaternion from gyroscope
		vx=2*(Quad_Buf[Quad_Delay][1]*Quad_Buf[Quad_Delay][3]-Quad_Buf[Quad_Delay][0]*Quad_Buf[Quad_Delay][2]);
		vy=2*(Quad_Buf[Quad_Delay][2]*Quad_Buf[Quad_Delay][3]+Quad_Buf[Quad_Delay][0]*Quad_Buf[Quad_Delay][1]);
		vz=1-2*(Quad_Buf[Quad_Delay][1]*Quad_Buf[Quad_Delay][1]+Quad_Buf[Quad_Delay][2]*Quad_Buf[Quad_Delay][2]);

		ex =(ay*vz-az*vy);
		ey =(az*vx-ax*vz);
		ez =(ax*vy-ay*vx);

		ex = constrain_float(ex, -1.0f, 1.0f);
		ey = constrain_float(ey, -1.0f, 1.0f);
		ez = constrain_float(ez, -1.0f, 1.0f);

		exInt +=ex*AHRS_KI* dt;
		eyInt +=ey*AHRS_KI* dt;
		ezInt +=ez*AHRS_KI* dt;

		exInt = constrain_float(exInt, -1.0f, 1.0f);
		eyInt = constrain_float(eyInt, -1.0f, 1.0f);
		ezInt = constrain_float(ezInt, -1.0f, 1.0f);

		gx = gx+exInt+AHRS_KP*ex;
		gy = gy+eyInt+AHRS_KP*ey;
		gz = gz+ezInt+AHRS_KP*ez;

		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0  * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0  * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0  * gz + q1 * gy - q2 * gx);

		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// delta = (dt * gx) * (dt * gx) + (dt * gy) * (dt * gy) + (dt * gz) * (dt * gz);
	// // Integrate rate of change of quaternion to yield quaternion
	// q0 = (1.0f - delta / 8.0f) * q0 + qDot1 * dt;
	// q1 = (1.0f - delta / 8.0f) * q1 + qDot2 * dt;
	// q2 = (1.0f - delta / 8.0f) * q2 + qDot3 * dt;
	// q3 = (1.0f - delta / 8.0f) * q3 + qDot4 * dt;

	q0 += qDot1 * (1.0f * dt);
	q1 += qDot2 * (1.0f * dt);
	q2 += qDot3 * (1.0f * dt);
	q3 += qDot4 * (1.0f * dt);


	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	T13 = 2 * (q1 * q3 - q0 * q2);
	if (fabsf(T13) > 1) 
	{
		if (T13 > 0) 
		{
			T13 = 1;
		}
		else 
		{
			T13 = -1;
		}
	}

	Theta_R = -asinf(T13);
	Theta = Theta_R * RAD2DEG;

	T23 = 2 * (q2 * q3 + q0 * q1);
	T33 = q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0;

	Gamma_R = atan2f(T23, T33);
	Gamma = Gamma_R * RAD2DEG;

	T12 = 2 * (q1 * q2 + q0 * q3);
	T11 = -q2 * q2 - q3 * q3 + q1 * q1 + q0 * q0;

	Phi_R = atan2f(T12, T11);
        
	if (Phi_R > PI) 
    {
	    Phi_R -= PI * 2;
	}
	else if (Phi_R < -PI) 
    {
	    Phi_R += PI * 2;
	}
        
	Phi = Phi_R * RAD2DEG;

	// 计算矩阵的其它元素
	T21 = 2 * (q1 * q2 - q0 * q3);
	T22 = q2 * q2 - q3 * q3 + q0 * q0 - q1 * q1;
	T31 = 2 * (q1 * q3 + q0 * q2);
	T32 = 2 * (q2 * q3 - q0 * q1);
}


void MadgwickAHR_Set_Beat(float p)
{
	beta = p;
}

#endif
//====================================================================================================
// END OF CODE
//====================================================================================================
