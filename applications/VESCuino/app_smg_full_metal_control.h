/*
 * app_smg_full_metal_control.h
 *
 *  Created on: 2018. 6. 21.
 *      Author: cdi
 */

#ifndef APPLICATIONS_VESCUINO_APP_SMG_FULL_METAL_CONTROL_H_
#define APPLICATIONS_VESCUINO_APP_SMG_FULL_METAL_CONTROL_H_

//
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "app.h"

//
typedef struct {
	// rotation velocity of wheel
	float omega_dot;
	float omega_dot_m;
	float omega_m1;
	float omega_dot_m1;
	float omega_dot_m1_des;
	float omega_m2;
	float omega_dot_m2;
	float omega_dot_m2_des;
	float omega_m3;
	float omega_dot_m3;
	float omega_dot_m3_des;
	float omega_m4;
	float omega_dot_m4;
	float omega_dot_m4_des;

	// tilt along x-axis
	float roll;
	float roll_dot;
	float roll_des;
	float roll_dot_des;
	float roll_err;
	float roll_dot_err;
	float roll_dt;

	// tilt along y-axis
	float pitch;
	float pitch_dot;
	float pitch_des;
	float pitch_dot_des;
	float pitch_err;
	float pitch_dot_err;
	float pitch_dt;

	//
	float pitch_int;

	// rotation along z-axis
	float yaw;
	float yaw_dot;
	float yaw_des;
	float yaw_dot_des;
	float yaw_err;
	float yaw_dot_err;
	float yaw_dt;

	// x-axis state
	float x;
	float x_dot;
	float x_dot_lpf;
	float x_des;
	float x_dot_des;
	float x_dot_des_lpf;
	float x_err;
	float x_dot_err;

	// y-axis state
	float y;
	float y_dot;
	float y_dot_lpf;
	float y_des;
	float y_dot_des;
	float y_dot_des_lpf;
	float y_err;
	float y_dot_err;

	// z-axis state
	float z;
	float z_dot;
	float z_dot_lpf;
	float z_des;
	float z_dot_des;
	float z_dot_des_lpf;
	float z_err;
	float z_dot_err;

	float gyro_x;
	float gyro_x_int;
	float gyro_y;
	float gyro_y_int;
	float gyro_z;
	float gyro_z_int;

	float imu_rate_hz;

	uint16_t imu_valid_cnt;

	// control state
	float control_rate_hz;
	float u_x;
	float u_y;
	float u_z;

	// current out
	float current_sensed_1;
	float current_1;
	float current_1_lpf;
	float current_sensed_2;
	float current_2;
	float current_2_lpf;
	float current_sensed_3;
	float current_3;
	float current_3_lpf;
	float current_sensed_4;
	float current_4;
	float current_4_lpf;

	// duty out
	float duty_1;
	float duty_1_lpf;
	float duty_2;
	float duty_2_lpf;
	float duty_3;
	float duty_3_lpf;
	float duty_4;
	float duty_4_lpf;

	//
	float cont_out_forward;
	float cont_out_translation;
	float cont_out_rotation;

	// Force and torque out
	float Fx;
	float Fy;
	float Tz;

	// remote sw
	uint8_t remote_sw[2];
	uint8_t output_to_remote[2];

	// fsr sensor
	int fsr[2];

	//amg
	uint8_t amg_linear_actuator_position_flag;	// 0:unknown, 1:initial, 2:home, 3:end
} CONT_VAR_MG;

//
typedef struct {
	//
	float dt;
	uint32_t ms;
	uint32_t smg_control_freq;
	bool en_flag;
	bool init_flag;
	bool bal_control_ready_flag;
	uint8_t bal_control_ready_state;
	bool bal_control_flag;
	bool brake_flag;//
	bool brake_flag_xbox;//
	bool brake_flag_nunchuck;//
	bool brake_flag_rsw;//
	uint8_t error_flag;
	bool spillover_filter_flag;
	bool current_limit_flag;
	uint8_t smg_control_mode;
	uint32_t duration;
	//uint8_t ground_imu_lpf_flag;

	//gains
	float spillover_filter_hz;
	float deadzone_current;
	float current_limit;
	float LQR_X[4];
	float LQR_Y[4];
	float LQR_Z[4];

	//can
	bool status_update_flag;
	bool smg_periodic_status_print_flag;
	bool smg_nrf_status_print_flag;
	char can_status_str[500];
	uint16_t cs_ind;

	//mcconf
	char mcconf_status_str[500];
	uint16_t ms_ind;
} PARAM_MG;

typedef enum {
	ERROR_NONE = 0,
	ERROR_IMU_WHOAMI_FIRST_MCU,
	ERROR_IMU_WHOAMI_SECOND_MCU,
	ERROR_IMU_I2C_RATE,
	ERROR_IMU_POSTPROCESS_RATE,
	ERROR_IMU_VALID_CNT,
	ERROR_TILT_ANGLE_OVER,
	ERROR_TILT_RATE_OVER,
	ERROR_MG_PROTO3_RATE
} error_flags;

void app_mg_set_error_flag(uint8_t error_code);

#endif /* APPLICATIONS_VESCUINO_APP_SMG_FULL_METAL_CONTROL_H_ */
