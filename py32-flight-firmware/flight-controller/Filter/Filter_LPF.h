#ifndef _FILTER_LPF_H_
#define _FILTER_LPF_H_
#include <stdint.h>

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


float Filter_LPF1(float hz, float time_step, float input, float *output);
float Filter_LPF2_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void  Filter_LPF2_SetCutoffFrequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
#endif


