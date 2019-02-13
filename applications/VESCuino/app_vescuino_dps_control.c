/*
 * app_vescuino_dps_control.c
 *
 *  Created on: 2018. 9. 12.
 *      Author: cdi
 */

#include "app.h"
#include "app_vescuino_dps_control.h"

#define USE_DPS_DT_PRINT	false

// Global Variable
static float dps_now;
static float deg_now;
static float deg_total;
static float dps_target;
static float dps_enc_offset_deg;
static int dps_set_flag = 0;
static int count = 0;

//
static THD_FUNCTION(dps_control_thread, arg);
static THD_WORKING_AREA(dps_control_thread_wa, 1024);

//
void app_vescuino_set_dps_thread_start(void)
{
	chThdCreateStatic(dps_control_thread_wa, sizeof(dps_control_thread_wa), NORMALPRIO, dps_control_thread, NULL);
	debug_printf("\r\nSET_DPS Thread Activated");
}

void app_vescuino_dps_set_enc_offset(float offset)
{
	dps_enc_offset_deg = offset;
}

float app_vescuino_dps_get_enc_offset(void)
{
	return dps_enc_offset_deg;
}

//
float genProfile(float *deg_tacho, float *deg_prof, float *dps_prof, float dps_goal, const float Amax, const float dt)
{
	float dv;
	float ds;

	if(*dps_prof == dps_goal) {
		ds = *dps_prof*dt;
		*deg_prof += ds;
		while(*deg_prof > 360.) {
			*deg_prof -= 360.;
		}
		while(*deg_prof < 0.) {
			*deg_prof += 360.;
		}
		*deg_tacho += ds;
		return ds;
	}

	if(dps_goal > *dps_prof) {
		dv = Amax*dt;
	}
	else {
		dv = -Amax*dt;
	}

	if(fabs(dv) > fabs(dps_goal - *dps_prof)) {
		dv = dps_goal - *dps_prof;
	}

	ds = *dps_prof*dt + 0.5*dv*dt;
	*dps_prof += dv;
	*deg_prof += ds;

	while(*deg_prof > 360.) {
		*deg_prof -= 360.;
	}
	while(*deg_prof < 0.) {
		*deg_prof += 360.;
	}
	*deg_tacho += ds;
	return ds;
}

void app_vescuino_set_dps(float dps)
{
	//debug_printf("\r> set_dps, dps_target=%.2f, dps_set_flag=%d, count=%d\r\n", (double)dps, dps_set_flag, count);

	dps_target = dps;
	// run this every first connection of dps control
	if(dps_set_flag==0) {
		deg_now = mc_interface_get_pid_pos_now();
		// reset offset angle when the hall/enc hybrid switch is already switched
		if(encoder_is_hall_enc_switched()) dps_enc_offset_deg = 0.;
	}
	dps_set_flag = 1;
	count = 0;
}

// DPS Control Thread
static THD_FUNCTION(dps_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("dps_control_thread");
	chThdSleepMilliseconds(1000);

	// time variables
	static systime_t time_start;
	static systime_t time_prev;
	static systime_t time_duration;
	uint32_t duration;

	int cnt = 0;

	for(;;) {
		// Timer implementation
		time_prev = time_start;
		time_start = chVTGetSystemTime();
		time_duration = time_start - time_prev;
		duration = ST2US(time_duration);	// usec

		genProfile(&deg_total, &deg_now, &dps_now, dps_target, 20000.0, 0.0001);

		if(dps_set_flag) {
			mc_interface_set_pid_pos(deg_now - dps_enc_offset_deg);
			timeout_reset();
		}

		// timeout 0.5s
		if(count>=5000 && dps_set_flag==1)	{
			count = 0;
			dps_set_flag = 0;
			dps_target = 0;
			dps_enc_offset_deg = 0;	// reset hall/enc hybrid offset for next dps control
			debug_printf("\r> dps_control timeout\r\n");
		}

		if(cnt>=10000 && dps_set_flag==1) {
			cnt = 0;
			if(USE_DPS_DT_PRINT) debug_printf("\r\n> dps_control, dt:%d (usec)", duration);
		}

		// Loop Time Manage
		count++;
		cnt++;
		chThdSleepMicroseconds(100);	// 10khz
	}
}
