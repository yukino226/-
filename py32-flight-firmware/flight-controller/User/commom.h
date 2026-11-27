#ifndef __COMMOM_H__
#define __COMMOM_H__
#include <stdint.h>

#define MPU_MPU6887    0
#define MPU_ICM42627   1
#define MPU_SELECT     MPU_ICM42627


#define DEG2RAD		       (0.0174533f)
#define DEG2RAD2	       (0.00872665f)
#define RAD2DEG            (57.29578f)


#define GRAVITY_MSS      9.80665f
#define PI               3.1415926535898f

#define M_PI_4 (0.78539816339744830962f)
#define dTnav  (0.01f)
#define ACC_VELOCITY_DT         (0.01f)

#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) :(x) ) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) )*( (in) - (out) )) //Ò»½×µÍÍ¨ÂË²¨

float Angle_Ceiling(float fdata,float max);
float Float_Ceiling(float fdata,float max);
int16_t S16_Ceiling(int16_t sdata,int16_t max);
float constrain_float(float amt, float low, float high);
float constrain(float value, float min, float max);
int32_t constrain_s32(int32_t amt, int32_t low, int32_t high);
int16_t constrain_s16(int16_t amt, int16_t low, int16_t high);
float wrap_180f(float error);
int32_t wrap_180(int32_t error);
float invSqrt(float x);
float fast_atan(float x);
#endif
