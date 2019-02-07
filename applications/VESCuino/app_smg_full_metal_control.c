/*
 * app_smg_full_metal_control.c
 *
 *  Created on: 21 June, 2018
 *      Author: cdi
 */

//
#include "app_smg_full_metal_control.h"

// SMG Full Metal Version. 20180621
#define MOTOR_CH1		0
#define MOTOR_CH2		1
#define MOTOR_CH3		2
#define MOTOR_CH4		3

// Motor Dependent Conversions
#define TORQUE2DUTY		0.38/42.	// [Rm/Kt]*[1duty/Volt]
#define POLEPAIR		10.0	// 4inch_m1(10.0), 5inch_electraCart(10.0), 6.5inch_jinwon(15 or 7.5?)
#define ERPM2RPM		1/POLEPAIR
#define ERPM2RPS		1/POLEPAIR*2.*M_PI/60.0
#define MOTOR_KT	    0.262	// Motor Torque Constant. 4inch_m1(0.262)
#define ROW          	0.0638   // radius of wheel [m] - Deview Beta(0.085), mg(2017-04-28_0.0638)
#define DCW             0.470   // distance from center to wheel - Deview Beta(0.470)
#define DCW_X			0.285	// distance from center to wheel of mg along x
#define DCW_Y			0.160   // distance from center to wheel of mg along y

//
static CONT_VAR_MG cont;
static PARAM_MG mg_param;
static nunchuk_data_norm chuk_d_norm;

static uint32_t loop_cnt_smg;
static float mg_rps[4] = {0.};
static float mg_spd[3] = {0.};
static float mg_pos[3] = {0.};
static float mg_curr[4] = {0.};
static float mg_dc[4] = {1.0, -1.0, 1.0, -1.0};	// Wheel Direction Correction

//
static THD_FUNCTION(smg_full_metal_control_thread, arg);
static THD_WORKING_AREA(smg_full_metal_control_thread_wa, 1024);

/*
 * SMG Full Metal Version Control Program
 */
// cdi
void mg_tilt_data_process(CONT_VAR_MG *con)
{
	// quaternion filter data
	con->roll = -imu_data.euler_x*DEG2RAD;
	con->roll_dot = imu_data.gyro_x*DEG2RAD;
	con->pitch = imu_data.euler_y*DEG2RAD;
	con->pitch_dot = imu_data.gyro_y*DEG2RAD;
	con->yaw = imu_data.euler_z*DEG2RAD;
	con->yaw_dot = imu_data.gyro_z*DEG2RAD;
	con->imu_rate_hz = imu_data.rate_hz;

	con->imu_valid_cnt = app_get_valid_cnt();

	//gyro integration in body coordinate
	app_util_cdi_integral(con->gyro_x, &con->gyro_x_int, mg_param.dt);
	app_util_cdi_integral(con->gyro_y, &con->gyro_y_int, mg_param.dt);
	app_util_cdi_integral(con->gyro_z, &con->gyro_z_int, mg_param.dt);
}

void app_mg_set_error_flag(uint8_t error_code)
{
	mg_param.error_flag = error_code;
}

float mg_get_erpm_from_can(uint8_t can_id)
{
	// erpm
	float m_ch_rpm;
	can_status_msg *m_ch_st_msg;
	m_ch_st_msg = comm_can_get_status_msg_id(can_id);
	m_ch_rpm = m_ch_st_msg->rpm;

	return m_ch_rpm;
}

float mg_get_rps_from_can(uint8_t can_id)
{
	// rps
	float m_ch_rps;
	can_status_msg *m_ch_st_msg;
	m_ch_st_msg = comm_can_get_status_msg_id(can_id);
	m_ch_rps = m_ch_st_msg->rps;

	return m_ch_rps;
}

//cdi
void mg_speed_data_process(CONT_VAR_MG *con)
{
	// Calculate rpm
	mg_rps[0] = (float)( ( (int32_t)(mc_interface_get_rpm()) )*ERPM2RPS );	// Check POLEPAIR Number
	mg_rps[1] = mg_get_rps_from_can(MOTOR_CH2);
	mg_rps[2] = mg_get_rps_from_can(MOTOR_CH3);
	mg_rps[3] = mg_get_rps_from_can(MOTOR_CH4);

	// Calculate rad/sec
	con->omega_dot_m1 = mg_dc[0]*mg_rps[0];
	con->omega_dot_m2 = mg_dc[1]*mg_rps[1];
	con->omega_dot_m3 = mg_dc[2]*mg_rps[2];
	con->omega_dot_m4 = mg_dc[3]*mg_rps[3];

	// Calculate rad
	app_util_cdi_integral(con->omega_dot_m1, &con->omega_m1, mg_param.dt);
	app_util_cdi_integral(con->omega_dot_m2, &con->omega_m2, mg_param.dt);
	app_util_cdi_integral(con->omega_dot_m3, &con->omega_m3, mg_param.dt);
	app_util_cdi_integral(con->omega_dot_m4, &con->omega_m4, mg_param.dt);

	// Calculate vx, vy, wz
	// Ref. Omnidirectional Mobile Robot – Design and Implementation (Ioan Doroftei, Victor Grosu and Veaceslav Spinu)
	con->x_dot = mg_spd[0] = ROW/4.*(con->omega_dot_m1 + con->omega_dot_m2 + con->omega_dot_m3 + con->omega_dot_m4);
	con->y_dot = mg_spd[1] = ROW/4.*(-con->omega_dot_m1 + con->omega_dot_m2 + con->omega_dot_m3 - con->omega_dot_m4);
	con->z_dot = mg_spd[2] = ROW/4./(DCW_X+DCW_Y)*(-con->omega_dot_m1 + con->omega_dot_m2 - con->omega_dot_m3 + con->omega_dot_m4);

	//app_util_cdi_lowpass_filter(con->x_dot, &con->x_dot_lpf, mg_param.velocity_lowpass_filter_hz, mg_param.dt);
	//app_util_cdi_lowpass_filter(con->y_dot, &con->y_dot_lpf, mg_param.velocity_lowpass_filter_hz, mg_param.dt);
	//app_util_cdi_lowpass_filter(con->z_dot, &con->z_dot_lpf, mg_param.velocity_lowpass_filter_hz, mg_param.dt);

	// Calculate x, y, z
	app_util_cdi_integral(con->x_dot, &con->x, mg_param.dt);
	app_util_cdi_integral(con->y_dot, &con->y, mg_param.dt);
	app_util_cdi_integral(con->z_dot, &con->z, mg_param.dt);

	mg_pos[0] = con->x;	// m
	mg_pos[1] = con->y;	// m
	mg_pos[2] = con->z;	// rad
}

float mg_get_current_from_can(uint8_t can_id)
{
	// erpm
	float m_ch_curr;
	can_status_msg *m_ch_st_msg;
	m_ch_st_msg = comm_can_get_status_msg_id(can_id);
	m_ch_curr = m_ch_st_msg->current;

	return m_ch_curr;
}

void mg_current_data_process(CONT_VAR_MG *con)
{
	mg_curr[0] = con->current_sensed_1 = mc_interface_read_reset_avg_motor_current();
	mg_curr[1] = con->current_sensed_2 = mg_get_current_from_can(MOTOR_CH2);
	mg_curr[2] = con->current_sensed_3 = mg_get_current_from_can(MOTOR_CH3);
	mg_curr[3] = con->current_sensed_4 = mg_get_current_from_can(MOTOR_CH4);
}

void mg_all_current_limit(CONT_VAR_MG *con, float limit_curr)
{
	app_util_cdi_limit(&con->current_1_lpf, limit_curr);
	app_util_cdi_limit(&con->current_2_lpf, limit_curr);
	app_util_cdi_limit(&con->current_3_lpf, limit_curr);
	app_util_cdi_limit(&con->current_4_lpf, limit_curr);
}

void mg_all_current_spillover_filter(CONT_VAR_MG *con, float freq_hz)
{
	app_util_cdi_lowpass_filter(con->current_1, &con->current_1_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->current_2, &con->current_2_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->current_3, &con->current_3_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->current_4, &con->current_4_lpf, freq_hz, mg_param.dt);
}

void mg_all_current_deadzone_compensator(CONT_VAR_MG *con, float deadzone_current)
{
	app_util_cdi_deadzone_compensation(&con->current_1, deadzone_current);
	app_util_cdi_deadzone_compensation(&con->current_2, deadzone_current);
	app_util_cdi_deadzone_compensation(&con->current_3, deadzone_current);
	app_util_cdi_deadzone_compensation(&con->current_4, deadzone_current);
}

void mg_all_duty_spillover_filter(CONT_VAR_MG *con, float freq_hz)
{
	app_util_cdi_lowpass_filter(con->duty_1, &con->duty_1_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->duty_2, &con->duty_2_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->duty_3, &con->duty_3_lpf, freq_hz, mg_param.dt);
	app_util_cdi_lowpass_filter(con->duty_4, &con->duty_4_lpf, freq_hz, mg_param.dt);
}

void mg_all_duty_limit(CONT_VAR_MG *con, float limit_duty)
{
	app_util_cdi_limit(&con->duty_1_lpf, limit_duty);
	app_util_cdi_limit(&con->duty_2_lpf, limit_duty);
	app_util_cdi_limit(&con->duty_3_lpf, limit_duty);
	app_util_cdi_limit(&con->duty_4_lpf, limit_duty);
}

//
void mg_all_current_out(CONT_VAR_MG *con, bool sof_select)
{
	if(sof_select==1)
	{
		mc_interface_set_current(mg_dc[0]*con->current_1_lpf);
		comm_can_set_current((uint8_t)MOTOR_CH2, mg_dc[1]*con->current_2_lpf);
		comm_can_set_current((uint8_t)MOTOR_CH3, mg_dc[2]*con->current_3_lpf);
		comm_can_set_current((uint8_t)MOTOR_CH4, mg_dc[3]*con->current_4_lpf);
	}
	else
	{
		mc_interface_set_current(mg_dc[0]*con->current_1);
		comm_can_set_current((uint8_t)MOTOR_CH2, mg_dc[1]*con->current_2);
		comm_can_set_current((uint8_t)MOTOR_CH3, mg_dc[2]*con->current_3);
		comm_can_set_current((uint8_t)MOTOR_CH4, mg_dc[3]*con->current_4);
	}

	timeout_reset();	// needed after every 'mc_interface_set_current()'
}

//
void mg_all_duty_out(CONT_VAR_MG *con, bool sof_select)
{
	if(sof_select==1)
	{
		mc_interface_set_duty(mg_dc[0]*con->duty_1_lpf);
		comm_can_set_duty((uint8_t)MOTOR_CH2, mg_dc[1]*con->duty_2_lpf);
		comm_can_set_duty((uint8_t)MOTOR_CH3, mg_dc[2]*con->duty_3_lpf);
		comm_can_set_duty((uint8_t)MOTOR_CH4, mg_dc[3]*con->duty_4_lpf);
	}
	else
	{
		mc_interface_set_duty(mg_dc[0]*con->duty_1);
		comm_can_set_duty((uint8_t)MOTOR_CH2, mg_dc[1]*con->duty_2);
		comm_can_set_duty((uint8_t)MOTOR_CH3, mg_dc[2]*con->duty_3);
		comm_can_set_duty((uint8_t)MOTOR_CH4, mg_dc[3]*con->duty_4);
	}

	timeout_reset();	// needed after every 'mc_interface_set_current()'
}

//
void mg_all_release(void)
{
	mc_interface_set_current(0.);
	comm_can_set_current((uint8_t)MOTOR_CH2, 0.);
	comm_can_set_current((uint8_t)MOTOR_CH3, 0.);
	comm_can_set_current((uint8_t)MOTOR_CH4, 0.);

	timeout_reset();	// needed after every 'mc_interface_set_current()'
}

void mg_all_brake(float curr)
{
	mc_interface_set_brake_current(curr);
	comm_can_set_current_brake((uint8_t)MOTOR_CH2, curr);
	comm_can_set_current_brake((uint8_t)MOTOR_CH3, curr);
	comm_can_set_current_brake((uint8_t)MOTOR_CH4, curr);

	timeout_reset();	// needed after every 'mc_interface_set_current()'
}

void mg_ctrl_var_reset(CONT_VAR_MG *con)
{
	// reset all control variables
	if(mg_param.init_flag == 0)
	{
		memset(con, 0.0, sizeof(CONT_VAR_MG));
		mg_param.init_flag = 1;
	}
}

void mg_all_control_out(CONT_VAR_MG *con)
{
	// duty or current
	if(mg_param.smg_control_mode==0)	// duty
	{
		// duty out
		con->duty_1 = con->cont_out_forward - con->cont_out_rotation;
		con->duty_2 = con->cont_out_forward + con->cont_out_rotation;
		con->duty_3 = con->cont_out_forward + con->cont_out_rotation;
		con->duty_4 = con->cont_out_forward - con->cont_out_rotation;

		// duty control all motor
		mg_all_duty_spillover_filter(con, mg_param.spillover_filter_hz);
		mg_all_duty_limit(con, 0.95f);
		mg_all_duty_out(con, mg_param.spillover_filter_flag);
	}
	else if(mg_param.smg_control_mode==1)	// current
	{
		// current out
		con->current_1 = con->cont_out_forward - con->cont_out_rotation;
		con->current_2 = con->cont_out_forward + con->cont_out_rotation;
		con->current_3 = con->cont_out_forward + con->cont_out_rotation;
		con->current_4 = con->cont_out_forward - con->cont_out_rotation;

		//current control all motor
		mg_all_current_deadzone_compensator(con, mg_param.deadzone_current);
		mg_all_current_spillover_filter(con, mg_param.spillover_filter_hz);
		mg_all_current_limit(con, mg_param.current_limit);
		mg_all_current_out(con, mg_param.spillover_filter_flag);	// important!: Set motor min current(1.0A -> 0.01A)
	}
}

void smg_balance_control_lqr_3d_mipc_full_metal_version(CONT_VAR_MG *con)
{
	con->x_dot_des = 3.0*(chuk_d_norm.js_x);
	con->y_dot_des = 0.5*(chuk_d_norm.js_y);

	app_util_cdi_lowpass_filter(con->x_dot_des, &con->x_dot_des_lpf, 1.0, mg_param.dt);
	app_util_cdi_lowpass_filter(con->y_dot_des, &con->y_dot_des_lpf, 1.0, mg_param.dt);

	//
	app_util_cdi_integral(con->x_dot_des, &con->x_des, mg_param.dt);
	app_util_cdi_integral(con->y_dot_des, &con->y_des, mg_param.dt);

	// Balancing controller
	if(mg_param.bal_control_flag==0)
	{
		con->pitch_int = 0;

		// remote control
		con->cont_out_forward = (1.0)*(con->x_dot_des_lpf - con->x_dot_lpf);//-(mg_param.LQR_Y[0]*(con->x_des - con->x) + mg_param.LQR_Y[1]*(con->x_dot_des_lpf - con->x_dot_lpf));
		con->cont_out_rotation = (5.0)*(con->y_dot_des_lpf - con->y_dot_lpf);//-(mg_param.LQR_Z[0]*(con->y_des - con->y) + mg_param.LQR_Z[1]*(con->y_dot_des_lpf - con->y_dot_lpf) );
	}
	else
	{
		//	pitch integral for fast braking
		//app_util_cdi_highpass_integral_filter(con->pitch, &con->pitch_int, mg_param.pitch_integral_cuttoff_hz, mg_param.dt);

		con->cont_out_forward = (mg_param.LQR_X[1]*(-con->x_dot_lpf) + mg_param.LQR_X[2]*(-con->pitch) + mg_param.LQR_X[3]*(-con->pitch_dot));// + mg_param.LQR_X[0]*(-con->pitch_int);
		//con->cont_out_rotation = (mg_param.LQR_Z[2]*(-(con->roll)) + mg_param.LQR_Z[3]*(-con->roll_dot));

#ifdef USE_SPEED_LIMIT_ALARM
		if(fabs(con->x_dot_lpf*MPS2KMH)>27)
		{
			cont.output_to_remote[0] = 1;
		}
		else
		{
			cont.output_to_remote[0] = 0;
		}
		write_to_remote(cont.output_to_remote);
#endif
	}

	if(fabs(con->pitch_dot*RAD2DEG)>200)
	{
		app_mg_set_error_flag(ERROR_TILT_RATE_OVER);	// angle_rate over
	}

	if(fabs(con->pitch*RAD2DEG)>15)
	{
		app_mg_set_error_flag(ERROR_TILT_ANGLE_OVER);	// angle over
	}

	if(con->imu_valid_cnt<100)
	{
		app_mg_set_error_flag(ERROR_IMU_VALID_CNT);	// angle over
	}

#ifdef USE_ERROR_BRAKE
	if(mg_param.error_flag!=0)
	{
		mg_param.brake_flag = 1;
	}
#endif
}

void app_smg_validation_can(void)
{
	// buffer rest
	memset(mg_param.can_status_str, 0, sizeof(char [500]));
	mg_param.cs_ind = 0;

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++)
	{
		can_status_msg *msg = comm_can_get_status_msg_index(i);

		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0)
		{
			sprintf(&mg_param.can_status_str[mg_param.cs_ind], "ID : %i / ", msg->id);
			mg_param.cs_ind += strlen(&mg_param.can_status_str[mg_param.cs_ind]);
			sprintf(&mg_param.can_status_str[mg_param.cs_ind], "RX Time : %i / ", (int)msg->rx_time);
			mg_param.cs_ind += strlen(&mg_param.can_status_str[mg_param.cs_ind]);
			sprintf(&mg_param.can_status_str[mg_param.cs_ind], "Age (msec) : %.2f\n", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
			mg_param.cs_ind += strlen(&mg_param.can_status_str[mg_param.cs_ind]);
		}
	}

	if(strlen(&mg_param.can_status_str[0]) == 0)
	{
		sprintf(&mg_param.can_status_str[0], "Can devices are not connected");
	}
}

void app_smg_print_status(uint8_t mode) {
	if(mode==2) {
		debug_printf("smg control loop freq:       %d\r\n", (int)mg_param.smg_control_freq);
		debug_printf("smg control loop dt:         %d(ms)\r\n", (int)(mg_param.dt*1000));
		debug_printf("smg control loop duration:   %d(usec)\r\n", (int)mg_param.duration);
		debug_printf("Brake Flag:                  %d\r\n", (bool)mg_param.brake_flag);
		debug_printf("Error Code:                  %d\r\n", mg_param.error_flag);
		debug_printf("Init. Flag:                  %d\r\n", (bool)mg_param.init_flag);
		debug_printf("Enable Flag:                 %d\r\n", (bool)mg_param.en_flag);
		debug_printf("Balancing Ready Flag:        %d\r\n", (bool)mg_param.bal_control_ready_flag);
		debug_printf("Balancing Ready State:       %d\r\n", (bool)mg_param.bal_control_ready_state);
		debug_printf("Balancing Flag:              %d\r\n", (bool)mg_param.bal_control_flag);
		debug_printf("SMG Status Update Flag:      %d\r\n"  , (bool)mg_param.status_update_flag);
		debug_printf("Bal. Control SOF:            %.2f\r\n", (double)mg_param.spillover_filter_hz);
		debug_printf("Bal. Control SOF On:         %d\r\n"  , (bool)mg_param.spillover_filter_flag);
		debug_printf("Current Limit:               %.2f\r\n", (double)mg_param.current_limit);
		debug_printf("Current Limit On:            %d\r\n"  , (bool)mg_param.current_limit_flag);
		debug_printf("LQR_X:     %.5f, %.5f, %.5f, %.5f\r\n", (double)mg_param.LQR_X[0], (double)mg_param.LQR_X[1], (double)mg_param.LQR_X[2], (double)mg_param.LQR_X[3]);
		debug_printf("LQR_Y:     %.5f, %.5f, %.5f, %.5f\r\n", (double)mg_param.LQR_Y[0], (double)mg_param.LQR_Y[1], (double)mg_param.LQR_Y[2], (double)mg_param.LQR_Y[3]);
		debug_printf("LQR_Z:     %.5f, %.5f, %.5f, %.5f\r\n", (double)mg_param.LQR_Z[0], (double)mg_param.LQR_Z[1], (double)mg_param.LQR_Z[2], (double)mg_param.LQR_Z[3]);
		debug_printf("--------------------------------------------\r\n");
		debug_printf("MCCONF SRAM Size:            %d Byte\r\n", (int16_t)sizeof(mc_configuration));
		debug_printf("APPCONF SRAM Size:           %d Byte\r\n", (int16_t)sizeof(app_configuration));
		debug_printf("--------------------------------------------\r\n");
		app_smg_validation_can();
		debug_printf("Can Devs: \r\n");
		debug_printf("%s\r\n", &mg_param.can_status_str[0]);
		debug_printf("--------------------------------------------\r\n");
	}
	else {
		mg_param.smg_periodic_status_print_flag = mode;
	}
}

void app_smg_nrf_values_print(int flag)
{
	mg_param.smg_nrf_status_print_flag = flag;
}

void app_smg_print_status_periodic(void) {
	debug_printf("\r\n%d - [IMU] roll:%.3f, pitch:%.3f, yaw:%.3f", loop_cnt_smg, (double)imu_data.euler_x, (double)imu_data.euler_y, (double)imu_data.euler_z);
	debug_printf("\r\n%d - [RPS] FL:%.3f, FR:%.3f / RL:%.3f, RR:%.3f", loop_cnt_smg, (double)cont.omega_dot_m1, (double)cont.omega_dot_m2, (double)cont.omega_dot_m3, (double)cont.omega_dot_m4);
	debug_printf("\r\n%d - [MPS] x_dot:%.3f, y_dot:%.3f, z_dot:%.3f", loop_cnt_smg, (double)cont.x_dot, (double)cont.y_dot, (double)cont.z_dot);
}

void app_smg_param_update(void)
{
	// Default Parameter is defined at appconf_custom.h
	//
	mg_param.smg_control_mode = appconf->app_custom.smg_control_mode;
	mg_param.smg_control_freq = appconf->app_custom.smg_control_freq;
	mg_param.spillover_filter_flag = appconf->app_custom.spillover_filter_flag;
	mg_param.spillover_filter_hz = appconf->app_custom.spillover_filter_hz;
	mg_param.current_limit_flag = appconf->app_custom.current_limit_flag;
	mg_param.current_limit = appconf->app_custom.current_limit;
	mg_param.deadzone_current = appconf->app_custom.deadzone_current;

	mg_param.LQR_X[0] = appconf->app_custom.LQR_X[0];
	mg_param.LQR_X[1] = appconf->app_custom.LQR_X[1];
	mg_param.LQR_X[2] = appconf->app_custom.LQR_X[2];
	mg_param.LQR_X[3] = appconf->app_custom.LQR_X[3];

//	mg_param.LQR_Y[0] = appconf->app_custom.LQR_Y[0];
//	mg_param.LQR_Y[1] = appconf->app_custom.LQR_Y[1];
//	mg_param.LQR_Y[2] = appconf->app_custom.LQR_Y[2];
//	mg_param.LQR_Y[3] = appconf->app_custom.LQR_Y[3];
//
//	mg_param.LQR_Z[0] = appconf->app_custom.LQR_Z[0];
//	mg_param.LQR_Z[1] = appconf->app_custom.LQR_Z[1];
//	mg_param.LQR_Z[2] = appconf->app_custom.LQR_Z[2];
//	mg_param.LQR_Z[3] = appconf->app_custom.LQR_Z[3];

	mg_param.status_update_flag = 1;
}

void app_smg_set_lqr_gain(float lqr_1, float lqr_2, float lqr_3, uint8_t axis)
{
	if(axis==0) {
		appconf->app_custom.LQR_X[1] = lqr_1;
		appconf->app_custom.LQR_X[2] = lqr_2;
		appconf->app_custom.LQR_X[3] = lqr_3;
		debug_printf("\r\nSet SMG LQR GAIN");
	    debug_printf("LQR_X: %.5f, %.5f, %.5f, %.5f\r\n", (double)appconf->app_custom.LQR_X[0], (double)appconf->app_custom.LQR_X[1], (double)appconf->app_custom.LQR_X[2], (double)appconf->app_custom.LQR_X[3]);
	}
//	else if(axis==1) {
//		appconf->app_custom.LQR_Y[1] = lqr_1;
//		appconf->app_custom.LQR_Y[2] = lqr_2;
//		appconf->app_custom.LQR_Y[3] = lqr_3;
//		debug_printf("\r\nSet SMG LQR GAIN");
//		debug_printf("LQR_Y: %.5f, %.5f, %.5f, %.5f\r\n", (double)appconf->app_custom.LQR_Y[0], (double)appconf->app_custom.LQR_Y[1], (double)appconf->app_custom.LQR_Y[2], (double)appconf->app_custom.LQR_Y[3]);
//	}
//	else if(axis==2) {
//		appconf->app_custom.LQR_Z[1] = lqr_1;
//		appconf->app_custom.LQR_Z[2] = lqr_2;
//		appconf->app_custom.LQR_Z[3] = lqr_3;
//		debug_printf("\r\nSet SMG LQR GAIN");
//		debug_printf("LQR_Z: %.5f, %.5f, %.5f, %.5f\r\n", (double)appconf->app_custom.LQR_Z[0], (double)appconf->app_custom.LQR_Z[1], (double)appconf->app_custom.LQR_Z[2], (double)appconf->app_custom.LQR_Z[3]);
//	}
	conf_general_store_app_configuration(appconf);
	app_smg_param_update();
	debug_printf("\r\n");
}

void app_smg_full_metal_thread_start(void)
{
	//
    app_smg_param_update();

	if(mg_param.smg_control_freq>=100 || mg_param.smg_control_freq<=1000)
	{
		chThdCreateStatic(smg_full_metal_control_thread_wa, sizeof(smg_full_metal_control_thread_wa), NORMALPRIO+3, smg_full_metal_control_thread, NULL);
	}
	debug_printf("\r\nSMG Full-Metal Version Control Thread Activated");
}

uint8_t app_mg_get_error_flag(void)
{
	return mg_param.error_flag;
}

// SMG Control Thread
static THD_FUNCTION(smg_full_metal_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("smg_full_metal_cont");
	chThdSleepMilliseconds(2000);

	// time variables
	static systime_t time_start;
	static systime_t time_prev;
	//static systime_t time_end;
	static systime_t time_duration;

	//Nunchuk Variables for Button
	static uint32_t bt_z_click_cnt = 0;
	static bool bt_z_hold_flag = 0;
	static uint32_t bt_c_click_cnt = 0;
	static bool bt_c_hold_flag = 0;

	// control variable init
	mg_param.dt = (float)1/mg_param.smg_control_freq;
	mg_param.ms = (uint32_t)(1000./mg_param.smg_control_freq);

	for(;;) {
		// Timer implementation
		time_prev = time_start;
		time_start = chVTGetSystemTime();
		time_duration = time_start - time_prev;
		mg_param.duration = ST2US(time_duration);	// usec
		//time_end = time_start + MS2ST(mg_param.ms);	// millisecond to system time

		if(mg_param.ms != (mg_param.duration/1000))	app_mg_set_error_flag(ERROR_MG_PROTO3_RATE);
		else	app_mg_set_error_flag(ERROR_NONE);

		// prepare state variables
		mg_tilt_data_process(&cont);
		mg_speed_data_process(&cont);	// after tilt_data_process
		mg_current_data_process(&cont);

		// ############## Button Control ##############
		// Using Nunchuk
		{
			app_nunchuk_get_normalized_data(&chuk_d_norm);

			// 'button C click' is Enable
			// 'button C hold' is balancing control ready
			if (chuk_d_norm.bt_c==0) {
				bt_c_hold_flag = 0;
				bt_c_click_cnt = 0;
			} else if (chuk_d_norm.bt_c==1) {
				if (bt_c_hold_flag == 0) bt_c_click_cnt++;

				// button C click
				if (bt_c_click_cnt == 1) {
					// Control On and init
					mg_param.en_flag = 1;
					mg_param.init_flag = 0;
					//mg_all_release();
				}

				// button C hold
				if(bt_c_click_cnt >= mg_param.smg_control_freq) {
					bt_c_hold_flag = 1;

					// Balancing Control ready
					if(mg_param.bal_control_ready_flag==0 && mg_param.bal_control_flag==0)
					{
						mg_param.bal_control_ready_flag = 1;
					}
					bt_c_click_cnt = 0;
				}
			}

			// 'button Z click' is Balancing control Off
			// 'button Z hold' is velocity feed-forward
			if (chuk_d_norm.bt_z==0) {
				bt_z_hold_flag = 0;
				bt_z_click_cnt = 0;
			} else if (chuk_d_norm.bt_z==1) {
				if (bt_z_hold_flag == 0) bt_z_click_cnt++;

				// button Z click
				if (bt_z_click_cnt == 1) {
					// Balancing Control off
					mg_param.bal_control_flag = 0;
					mg_param.bal_control_ready_flag = 0;
					mg_param.bal_control_ready_state = 0;
				}

				// button Z hold
				if(bt_z_click_cnt >= mg_param.smg_control_freq) {
					bt_z_hold_flag = 1;
				}
			}

			// 'Button C & Z Click' is reset
			if(chuk_d_norm.bt_c == 1 && chuk_d_norm.bt_z == 1)
			{
				// Control off and reset
				mg_param.en_flag = 0;
				mg_param.bal_control_flag = 0;
				mg_param.bal_control_ready_flag = 0;
				mg_param.bal_control_ready_state = 0;
				mg_ctrl_var_reset(&cont);
				mg_all_release();
			}
		}
		// ############## End of Button Control ##############

#ifdef USE_FSR_SENSOR
		cont.fsr[0] = ADC_Value[9];
		cont.fsr[1] = ADC_Value[10];
#endif

		// brake flag
		mg_param.brake_flag = chuk_d_norm.bt_z;

		// Balancing on control
		if(mg_param.bal_control_ready_flag == 1 && mg_param.bal_control_flag == 0)
		{
			if(mg_param.bal_control_ready_state == 0)
			{
				if(fabsf(cont.pitch) >= 3.0f*DEG2RAD)
				{
					mg_param.bal_control_ready_state = 1;
				}
			}
			else if(mg_param.bal_control_ready_state == 1)
			{
				if(fabsf(cont.pitch) <= 0.5f*DEG2RAD)
				{
					mg_param.bal_control_ready_state = 2;
				}
			}
		}
		if(mg_param.bal_control_ready_state == 2)
		{
			mg_param.bal_control_ready_state = 0;
			mg_param.bal_control_ready_flag = 0;
			mg_param.bal_control_flag = 1;
		}

		// On Control State
		if(mg_param.en_flag == 1)
		{
			if(mg_param.brake_flag==0)
			{
				smg_balance_control_lqr_3d_mipc_full_metal_version(&cont);

				// Control Out
				mg_all_control_out(&cont);
			}
			else if(mg_param.brake_flag == 1)
			{
				if(mg_param.brake_flag) mg_all_brake(8.0);	// max 10.A

				mg_param.bal_control_flag = 0;
				mg_param.bal_control_ready_flag = 0;
				mg_param.bal_control_ready_state = 0;
				mg_ctrl_var_reset(&cont);
			}
		}
		// Off Control State
		else
		{
			mg_all_release();
		}

		//
		if(mg_param.smg_periodic_status_print_flag)
		{
			if(app_run_every_x_hz(1, loop_cnt_smg, imu_data.rate_hz)) {
				app_smg_print_status_periodic();
			}
		}

		if(mg_param.smg_nrf_status_print_flag)
		{
			if(app_run_every_x_hz(1, loop_cnt_smg, imu_data.rate_hz)) {
				debug_printf("\r\n%d - Button <C: %d, Z: %d>, Joy <X: %.2f, Y: %.2f>", loop_cnt_smg,
																						(bool)chuk_d_norm.bt_c, (bool)chuk_d_norm.bt_z,
																						(double)chuk_d_norm.js_x, (double)chuk_d_norm.js_y);
			}
		}

		// Loop Time Manage
		loop_cnt_smg++;
		chThdSleepMilliseconds(mg_param.ms);

//		systime_t time_now = chVTGetSystemTime();
//		if(time_end>time_now) {
//			chThdSleepUntil(time_end);
//		}
	}

}
