#ifndef FOC_H
#define FOC_H

float limit_min(float input,float lim);
float limit_max(float input,float lim);
void SVPWM(float phi, float d, float q, float *d_u, float *d_v, float *d_w);
float Low_pass_filter(float input,float prop);
float cycle_diff(float diff, float cycle);
typedef struct
{
	float position;
	float speed;
	float torque_d;
	float torque_q;
	
}motor_control_value;
extern motor_control_value motor_control;
void ADC_1();



#endif
