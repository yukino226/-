//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef __MADGWICKAHRS_H__
#define __MADGWICKAHRS_H__
//----------------------------------------------------------------------------------------------------
// Variable declaration
extern float T13, T23, T33, T11, T12;
extern float T21, T22, T31, T32;
extern float HMC_DECLINATION,HMC_DECLINATION_R;
extern volatile float Theta_R, Gamma_R, Phi_R;
extern volatile float Theta,   Gamma,   Phi;

//---------------------------------------------------------------------------------------------------
// Function declarations
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void MadgwickAHRInit(void);
void MadgwickAHR_Set_Beat(float p);
void MadgwickAHR_ClearInt(void);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
