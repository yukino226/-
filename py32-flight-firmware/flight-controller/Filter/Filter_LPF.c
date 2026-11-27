#include "Filter_LPF.h"
#include "math.h"
#include "commom.h"

/*************************************************
2½×¶ÎµÍÍ¨ÂË²¨Æ÷
*************************************************/
float Filter_LPF2_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    /* »ñÈ¡×îÐÂx(n) */
    static int LPF_Cnt=0;
    Buffer->Input_Butter[2]=curr_input;
    if(LPF_Cnt>=100)
    {
      /* ButterworthÂË²¨ */
      Buffer->Output_Butter[2]=
        Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
      +Parameter->b[2] * Buffer->Input_Butter[0]
      -Parameter->a[1] * Buffer->Output_Butter[1]
      -Parameter->a[2] * Buffer->Output_Butter[0];
    }
    else
    {
      Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
      LPF_Cnt++;
    }
    /* x(n) ÐòÁÐ±£´æ */
    Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
    Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
    /* y(n) ÐòÁÐ±£´æ */
    Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
    Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
    return Buffer->Output_Butter[2];
}

// discrete low pass filter, cuts out the
// high frequency noise that can drive the controller crazy
//derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);
float Filter_LPF2_SetAlpha(int16_t cutoff_frequency, float time_step)
{
    // calculate alpha
    float lpf_alpha;
    float rc = 1/(2*PI*cutoff_frequency);
    lpf_alpha = time_step / (time_step + rc);
    return lpf_alpha;
}

void Filter_LPF2_SetCutoffFrequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
	float fr = sample_frequent / cutoff_frequent;
	float ohm = tanf(PI / fr);
	float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
	if (cutoff_frequent <= 0.0f) {
		// no filtering
		return;
	}
	LPF->b[0] = ohm * ohm / c;
	LPF->b[1] = 2.0f * LPF->b[0];
	LPF->b[2] = LPF->b[0];
        LPF->a[0]=1.0f;
	LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	LPF->a[2] = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

/*************************************************
1½×¶ÎµÍÍ¨ÂË²¨Æ÷
*************************************************/
float Filter_LPF1(float hz, float time_step, float input, float *output)
{
    float alpha = 1.0f / (1.0f + 1.0f / (hz * 3.14f * time_step));
    *output += alpha * (input - *output);
    return *output;
}
