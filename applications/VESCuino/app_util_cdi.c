/*
 * app_cdi_util.c
 *
 *  Created on: Apr 11, 2017
 *      Author: cdi
 */

#include "app.h"
#include <math.h>
#include <stdio.h>

void app_util_cdi_deadzone_compensation(float *in, float deadzone)
{
    float out = 0;

    if((double)fabs(*in) <= (double)deadzone) {
        out = 0.;
    } else {
        if(*in >= 0) out = (*in - deadzone)/(1-deadzone);
        else out = (*in + deadzone)/(1-deadzone);
    }

    *in = out;
}

double app_util_cdi_lowpass_filter_double(double in, double *out_prev, double hz, double dt)
{
	double out_lpf;
	out_lpf = (*out_prev + dt*(double)2.0f*(double)M_PI*hz*in)/((double)1.0 + dt*(double)2.0f*(double)M_PI*hz);
	*out_prev = out_lpf;

	return out_lpf;
}

float app_util_cdi_lowpass_filter(float in, float *out_prev, float hz, float dt)
{
	float out_lpf;
	out_lpf = (*out_prev + dt*2.0*M_PI*hz*in)/(1.0 + dt*2.0*M_PI*hz);
	*out_prev = out_lpf;

	return out_lpf;
}

double app_util_cdi_moving_average(double *moving_window, double *moving_sum, int *pos, int len, double nextNum)
{
	//Subtract the oldest number from the prev sum, add the new number
	*moving_sum = *moving_sum - moving_window[*pos] + nextNum;
	//Assign the nextNum to the position in the array
	moving_window[*pos] = nextNum;
	*pos = *pos + 1;
	if(*pos >= len) *pos = 0;
	//return the average
	return *moving_sum / len;
}

void app_util_cdi_integral(float in, float *out_last, float dt)
{
	float out;
	out = *out_last + in*dt;
	*out_last = out;
}

float app_util_cdi_highpass_integral_filter(float in, float *out_prev, float hz, float dt)
{
	float out_lpf;
	out_lpf = (*out_prev + dt*in)/(1.0 + dt*2.0*M_PI*hz);
	*out_prev = out_lpf;

	return out_lpf;
}

float app_util_cdi_time_derivative(float in, float *in_last, float dt)
{
	float out;
	out = (in - *in_last)/dt;
	*in_last = in;

	return out;
}

void app_util_cdi_limit(float *in, float limit)
{
    if(*in >= limit) {
    	*in = limit;
    } else if(*in <= -limit) {
    	*in = -limit;
    }
}

void app_conv_hex_to_ascii(uint8_t data_hex, char *data_ascii)
{
	sprintf(data_ascii,"%c", data_hex);
}
