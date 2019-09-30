/*
 * app_vescuino_dps_control.c
 *
 *  Created on: 2018. 9. 12.
 *      Author: cdi
 */

#include "app.h"
#include "app_vescuino_dps_control.h"

#define USE_DPS_DT_PRINT	false
#define DPS_DT				0.0001		// 10khz
#define DPS_max				25000.0		// 58000 erpm / 12 polepair = 4800 rpm * 6 = 29000 dps
#define V_max				60.0		// 60deg/1sample_period(at 100000rpm)

// Global Variable
static float v_prof;
static float v_ref;
static float s_prof;
static float s_actual;
static double tacho_prof;
static double tacho_actual;
static double deg_target;
static float deg_ref;
static float dps_enc_offset_deg;
static float dps_target;
static float dps_actual;
static int dps_set_flag = 0;
static int count = 0;
static int tacho_init=0;
				
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

float app_vescuino_dps_target(void)
{
	return dps_target;
}

float app_vescuino_dps_actual(void)
{
	return dps_actual;
}

float app_vescuino_dps_s_prof(void)
{
	return s_prof;
}

float app_vescuino_dps_s_actual(void)
{
	return s_actual;
}

double app_vescuino_dps_tacho_prof(void)
{
	return tacho_prof;
}

double app_vescuino_dps_tacho_actual(void)
{
	return tacho_actual;
}

double app_vescuino_dps_deg_target(void)
{
	return deg_target;
}

//
float genProfile(float v_ref, float Amax, float dt) //, const float Amax, const float dt)
{
	float da = 0;
	float dv = 0;
	float ds = 0;
	float s_temp = 0;
	float s_ref = 0;

	// Profile Deg
	if(v_ref == v_prof) {
		dv = 0;
	}
	else {
		da = (v_ref - v_prof)/dt;
		if(fabs(da) >= (double)Amax) {
			if(da>0) da = Amax;
			else 	 da = -Amax;
		}
	}
	dv = da*dt;
	ds = v_prof*dt + 0.5*dv*dt;

	v_prof += dv;
	s_prof += ds;
	while(s_prof > 360.) {
		s_prof -= 360.;
	}
	while(s_prof < 0.) {
		s_prof += 360.;
	}
	tacho_prof += (double)ds;

	// Actual Deg
	if(tacho_init==0) {
		tacho_actual = mc_interface_get_pid_pos_now();
		s_temp = s_actual = tacho_actual;
		tacho_init = 1;
	}
	else {
		s_temp = s_actual;
		s_actual = mc_interface_get_pid_pos_now();
		ds = s_actual - s_temp;
		tacho_actual += (double)ds;
	}
	dps_actual = ds/dt;
	if(ds > V_max) {
		tacho_actual -= (double)360.;
	}
	else if(ds < -V_max) {
		tacho_actual += (double)360.;
	}	

	s_ref = s_prof;
	return s_ref;	// position ref
}

void app_vescuino_set_dps(float dps)
{
	//debug_printf("\r> set_dps, dps_target=%.2f, dps_set_flag=%d, count=%d\r\n", (double)dps, dps_set_flag, count);

	dps_target = dps;
	// run this every first connection of dps control
	if(dps_set_flag==0) {
		s_prof = mc_interface_get_pid_pos_now();
		tacho_prof = mc_interface_get_pid_pos_now();
		// reset offset angle when the hall/enc hybrid switch is already switched
		if(encoder_is_hall_enc_switched()) dps_enc_offset_deg = 0.;
	}
	dps_set_flag = 1;
	count = 0;
}

void app_vescuino_set_goto(float deg)
{
	float dps_goto;
	deg_target = deg;

	dps_goto = 7.*(float)(deg_target - tacho_actual);	// 7:optimal, 8:overshoot little, 6.:no overshoot
	if(fabs(dps_goto) >= (double)DPS_max) {
		if(dps_goto>0)	dps_goto = DPS_max;
		else 	 		dps_goto = -DPS_max;
	}

	app_vescuino_set_dps(dps_goto);
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

		deg_ref = genProfile(dps_target, (float)(100000.0), (float)(duration/1000000.)); //, Amax, DPS_DT);

		if(dps_set_flag) {
			mc_interface_set_pid_pos(deg_ref - dps_enc_offset_deg);
			timeout_reset();
		}

		// timeout 0.5s
		if(count>=5000 && dps_set_flag==1)	{
			count = 0;
			dps_set_flag = 0;
			v_prof = 0;
			v_ref = 0;
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
