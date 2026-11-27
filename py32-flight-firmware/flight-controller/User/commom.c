#include <stdint.h>
#include "commom.h"
#include <math.h>
#include <stdlib.h>

int16_t  S16_Ceiling(int16_t sdata,int16_t max)
{
    if(abs(sdata)>max)
    {
        if(sdata>0)
        {
            sdata = max;
        }
        else
        {
            sdata = -max; 
        }
    }
    return sdata;
}

float Float_Ceiling(float fdata,float max)
{
    if(fabsf(fdata)>max)
    {
        if(fdata>0)
        {
            fdata = max;
        }
        else
        {
            fdata = -max; 
        }
    }
    return fdata;
}

float Angle_Ceiling(float fdata,float max)
{
    if(fdata<-180)
    {
        fdata += 360;
    }
    else if(fdata>180)
    {
        fdata -= 360;
    }
    
    if(fabsf(fdata)>max)
    {
        if(fdata>0)
        {
            fdata = max;
        }
        else
        {
            fdata = -max; 
        }
    }
    
    return fdata;
}


int32_t wrap_180(int32_t error)
{
	if (error > 18000)
        {
		error -= 36000;
	}
	if (error < -18000)
        {
		error += 36000;
	}
	return error;
}

float wrap_180f(float error)
{
	if (error > 180.0f) 
        {
		error -= 360.0f;
	}
	if (error < -180.0f) 
        {
		error += 360.0f;
	}
	return error;
}
float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

int16_t constrain_s16(int16_t amt, int16_t low, int16_t high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

int32_t constrain_s32(int32_t amt, int32_t low, int32_t high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

float constrain(float value, float min, float max)
{
	if (value < min) 
        {
		return min;
	}
	else if (value > max)
        {
		return max;
	}
	else 
        {
		return value;
	}
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float fast_atan(float x)
{
    return M_PI_4 * x - x * (fabsf(x) - 1) * (0.2447f + 0.0663f * fabsf(x));
}
