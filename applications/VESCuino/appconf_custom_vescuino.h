/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef APPCONF_APPCONF_CUSTOM_VESCUINO_H_
#define APPCONF_APPCONF_CUSTOM_VESCUINO_H_

// override app to use
#define APPCONF_APP_TO_USE					APP_CUSTOM	//APP_UART	//APP_CUSTOM

// Default app configuration
#ifndef APPCONF_CONTROLLER_ID
#define APPCONF_CONTROLLER_ID				2
#endif
#ifndef APPCONF_TIMEOUT_MSEC
#define APPCONF_TIMEOUT_MSEC				1000
#endif
#ifndef APPCONF_TIMEOUT_BRAKE_CURRENT
#define APPCONF_TIMEOUT_BRAKE_CURRENT		0.0
#endif
#ifndef APPCONF_SEND_CAN_STATUS
#define APPCONF_SEND_CAN_STATUS				APPCONF_CONTROLLER_ID ? true : false	// true, false
#endif
#ifndef APPCONF_SEND_CAN_STATUS_RATE_HZ
#define APPCONF_SEND_CAN_STATUS_RATE_HZ		500
#endif
#ifndef APPCONF_CAN_BAUD_RATE
#ifdef HW_VERSION_410
#define APPCONF_CAN_BAUD_RATE				CAN_BAUD_500K
#else
#define APPCONF_CAN_BAUD_RATE				CAN_BAUD_1M
#endif
#endif

// Custom Setting CDI
//APP
#ifndef CUSTOM_APP_MODE
#define CUSTOM_APP_MODE							0	// 0:VESCuino SPI, 1:SMG, 2:Differential Robot
#endif
//IMU
#ifndef CUSTOM_IMU_USE_MODE
#define CUSTOM_IMU_USE_MODE						0	// 0:off, 1:on, 2:1st, 2nd mpu on (SMG Mode), 3:1st&2nd SPI-HW(for SMG)
#endif
#ifndef CUSTOM_IMU_COMM_MODE_1ST
#define CUSTOM_IMU_COMM_MODE_1ST				0	// 1st mpu - 0:spi-sw, 1:i2c
#endif
#ifndef CUSTOM_IMU_COMM_MODE_2ND
#define CUSTOM_IMU_COMM_MODE_2ND				0	// 2nd mpu - 0:spi-hw, 1:i2c
#endif
#ifndef CUSTOM_IMU_SETUP_MODE_1ST
#define CUSTOM_IMU_SETUP_MODE_1ST				0	// 1st mpu - 0:basic, 1:fast, 2:full
#endif
#ifndef CUSTOM_IMU_SETUP_MODE_2ND
#define CUSTOM_IMU_SETUP_MODE_2ND				0	// 2nd mpu - 0:basic, 1:fast, 2:full
#endif
#ifndef CUSTOM_IMU_DOF_1ST
#define CUSTOM_IMU_DOF_1ST						0	// 1st mpu - 0:6DOF, 1:9DOF
#endif
#ifndef CUSTOM_IMU_DOF_2ND
#define CUSTOM_IMU_DOF_2ND						0	// 2nd mpu - 0:6DOF, 1:9DOF
#endif
#ifndef CUSTOM_IMU_QUATERNION_FILTER_MODE
#define CUSTOM_IMU_QUATERNION_FILTER_MODE		1	// 0:Madgwick, 1:Mahony
#endif
#ifndef CUSTOM_IMU_ASCALE
#define CUSTOM_IMU_ASCALE						3	// 0:2G, 1:4G, 2:8G, 3:16G
#endif
#ifndef CUSTOM_IMU_GSCALE
#define CUSTOM_IMU_GSCALE						3	// 0:250, 1:500, 2:1000, 3:2000
#endif
#ifndef CUSTOM_IMU_MSCALE
#define CUSTOM_IMU_MSCALE						1	// 0:MFS_14BIT, 1:MFS_16BITS
#endif
#ifndef CUSTOM_IMU_M_MODE
#define CUSTOM_IMU_M_MODE						1	// 0:8Hz(Mmode=0x02), 1:100Hz(Mmode=0x06)
#endif
#ifndef CUSTOM_IMU_RATE_FREQ
#define CUSTOM_IMU_RATE_FREQ					1000	// 1kHz
#endif
#ifndef CUSTOM_IMU_GYRO_BIAS_X
#define CUSTOM_IMU_GYRO_BIAS_X					0./1000.
#endif
#ifndef CUSTOM_IMU_GYRO_BIAS_Y
#define CUSTOM_IMU_GYRO_BIAS_Y					0./1000.
#endif
#ifndef CUSTOM_IMU_GYRO_BIAS_Z
#define CUSTOM_IMU_GYRO_BIAS_Z					0./1000.
#endif
#ifndef CUSTOM_IMU_ACC_BIAS_X
#define CUSTOM_IMU_ACC_BIAS_X					0./1000.
#endif
#ifndef CUSTOM_IMU_ACC_BIAS_Y
#define CUSTOM_IMU_ACC_BIAS_Y					0./1000.
#endif
#ifndef CUSTOM_IMU_ACC_BIAS_Z
#define CUSTOM_IMU_ACC_BIAS_Z					0./1000.
#endif
#ifndef CUSTOM_IMU_USE_ACCEL_BIAS_USER_1ST
#define CUSTOM_IMU_USE_ACCEL_BIAS_USER_1ST		0
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_X_1ST
#define CUSTOM_IMU_ACCEL_BIAS_USER_X_1ST		0./1000.
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_Y_1ST
#define CUSTOM_IMU_ACCEL_BIAS_USER_Y_1ST		0./1000.
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_Z_1ST
#define CUSTOM_IMU_ACCEL_BIAS_USER_Z_1ST		0./1000.
#endif
#ifndef CUSTOM_IMU_USE_ACCEL_BIAS_USER_2ND
#define CUSTOM_IMU_USE_ACCEL_BIAS_USER_2ND		0
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_X_2ND
#define CUSTOM_IMU_ACCEL_BIAS_USER_X_2ND		0./1000.
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_Y_2ND
#define CUSTOM_IMU_ACCEL_BIAS_USER_Y_2ND		0./1000.
#endif
#ifndef CUSTOM_IMU_ACCEL_BIAS_USER_Z_2ND
#define CUSTOM_IMU_ACCEL_BIAS_USER_Z_2ND		0./1000.
#endif
#ifndef CUSTOM_SMG_CONTROL_MODE
#define CUSTOM_SMG_CONTROL_MODE					1		// 0:duty, 1:current
#endif
#ifndef CUSTOM_SMG_CONTROL_FREQ
#define CUSTOM_SMG_CONTROL_FREQ					500.	// Hz
#endif
#ifndef CUSTOM_SMG_SPILLOVER_FILTER_FLAG
#define CUSTOM_SMG_SPILLOVER_FILTER_FLAG		1
#endif
#ifndef CUSTOM_SMG_SPILLOVER_FILTER_FREQ
#define CUSTOM_SMG_SPILLOVER_FILTER_FREQ		50.		// Hz
#endif
#ifndef CUSTOM_SMG_CURRENT_LIMIT_FLAG
#define CUSTOM_SMG_CURRENT_LIMIT_FLAG			1
#endif
#ifndef CUSTOM_SMG_CURRENT_LIMIT
#define CUSTOM_SMG_CURRENT_LIMIT				20.		// A
#endif

#ifndef CUSTOM_SMG_LQR_X_0
#define CUSTOM_SMG_LQR_X_0						0.
#endif
#ifndef CUSTOM_SMG_LQR_X_1
#define CUSTOM_SMG_LQR_X_1						-0.8	// basic:-0.5, intermediate:-0.8
#endif
#ifndef CUSTOM_SMG_LQR_X_2
#define CUSTOM_SMG_LQR_X_2						-150.	// basic:-100., intermediate:-150
#endif
#ifndef CUSTOM_SMG_LQR_X_3
#define CUSTOM_SMG_LQR_X_3						-15.	// basic:-10., intermediate:-15
#endif

// The default app is UART in case the UART port is used for
// firmware updates.
#ifndef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE					APP_UART
#endif

// PPM app configureation
#ifndef APPCONF_PPM_CTRL_TYPE
#define APPCONF_PPM_CTRL_TYPE				PPM_CTRL_TYPE_NONE
#endif
#ifndef APPCONF_PPM_PID_MAX_ERPM
#define APPCONF_PPM_PID_MAX_ERPM			15000
#endif
#ifndef APPCONF_PPM_HYST
#define APPCONF_PPM_HYST					0.15
#endif
#ifndef APPCONF_PPM_PULSE_START
#define APPCONF_PPM_PULSE_START				1.0
#endif
#ifndef APPCONF_PPM_PULSE_END
#define APPCONF_PPM_PULSE_END				2.0
#endif
#ifndef APPCONF_PPM_PULSE_CENTER
#define APPCONF_PPM_PULSE_CENTER			1.5
#endif
#ifndef APPCONF_PPM_MEDIAN_FILTER
#define APPCONF_PPM_MEDIAN_FILTER			true
#endif
#ifndef APPCONF_PPM_SAFE_START
#define APPCONF_PPM_SAFE_START				true
#endif
#ifndef APPCONF_PPM_THROTTLE_EXP
#define APPCONF_PPM_THROTTLE_EXP			0.0
#endif
#ifndef APPCONF_PPM_THROTTLE_EXP_BRAKE
#define APPCONF_PPM_THROTTLE_EXP_BRAKE		0.0
#endif
#ifndef APPCONF_PPM_THROTTLE_EXP_MODE
#define APPCONF_PPM_THROTTLE_EXP_MODE		THR_EXP_POLY
#endif
#ifndef APPCONF_PPM_RAMP_TIME_POS
#define APPCONF_PPM_RAMP_TIME_POS			0.3
#endif
#ifndef APPCONF_PPM_RAMP_TIME_NEG
#define APPCONF_PPM_RAMP_TIME_NEG			0.1
#endif
#ifndef APPCONF_PPM_MULTI_ESC
#define APPCONF_PPM_MULTI_ESC				false
#endif
#ifndef APPCONF_PPM_TC
#define APPCONF_PPM_TC						false
#endif
#ifndef APPCONF_PPM_TC_MAX_DIFF
#define APPCONF_PPM_TC_MAX_DIFF				3000.0
#endif

// ADC app configureation
#ifndef APPCONF_ADC_CTRL_TYPE
#define APPCONF_ADC_CTRL_TYPE				ADC_CTRL_TYPE_NONE
#endif
#ifndef APPCONF_ADC_HYST
#define APPCONF_ADC_HYST					0.15
#endif
#ifndef APPCONF_ADC_VOLTAGE_START
#define APPCONF_ADC_VOLTAGE_START			0.9
#endif
#ifndef APPCONF_ADC_VOLTAGE_END
#define APPCONF_ADC_VOLTAGE_END				3.0
#endif
#ifndef APPCONF_ADC_VOLTAGE_CENTER
#define APPCONF_ADC_VOLTAGE_CENTER			2.0
#endif
#ifndef APPCONF_ADC_VOLTAGE2_START
#define APPCONF_ADC_VOLTAGE2_START			0.9
#endif
#ifndef APPCONF_ADC_VOLTAGE2_END
#define APPCONF_ADC_VOLTAGE2_END			3.0
#endif
#ifndef APPCONF_ADC_USE_FILTER
#define APPCONF_ADC_USE_FILTER				true
#endif
#ifndef APPCONF_ADC_SAFE_START
#define APPCONF_ADC_SAFE_START				true
#endif
#ifndef APPCONF_ADC_CC_BUTTON_INVERTED
#define APPCONF_ADC_CC_BUTTON_INVERTED		false
#endif
#ifndef APPCONF_ADC_REV_BUTTON_INVERTED
#define APPCONF_ADC_REV_BUTTON_INVERTED		false
#endif
#ifndef APPCONF_ADC_VOLTAGE_INVERTED
#define APPCONF_ADC_VOLTAGE_INVERTED		false
#endif
#ifndef APPCONF_ADC_VOLTAGE2_INVERTED
#define APPCONF_ADC_VOLTAGE2_INVERTED		false
#endif
#ifndef APPCONF_ADC_THROTTLE_EXP
#define APPCONF_ADC_THROTTLE_EXP			0.0
#endif
#ifndef APPCONF_ADC_THROTTLE_EXP_BRAKE
#define APPCONF_ADC_THROTTLE_EXP_BRAKE		0.0
#endif
#ifndef APPCONF_ADC_THROTTLE_EXP_MODE
#define APPCONF_ADC_THROTTLE_EXP_MODE		THR_EXP_POLY
#endif
#ifndef APPCONF_ADC_RAMP_TIME_POS
#define APPCONF_ADC_RAMP_TIME_POS			0.3
#endif
#ifndef APPCONF_ADC_RAMP_TIME_NEG
#define APPCONF_ADC_RAMP_TIME_NEG			0.1
#endif
#ifndef APPCONF_ADC_MULTI_ESC
#define APPCONF_ADC_MULTI_ESC				false
#endif
#ifndef APPCONF_ADC_TC
#define APPCONF_ADC_TC						false
#endif
#ifndef APPCONF_ADC_TC_MAX_DIFF
#define APPCONF_ADC_TC_MAX_DIFF				3000.0
#endif
#ifndef APPCONF_ADC_UPDATE_RATE_HZ
#define APPCONF_ADC_UPDATE_RATE_HZ			500
#endif

// UART app
#ifndef APPCONF_UART_BAUDRATE
#define APPCONF_UART_BAUDRATE				115200
#endif

// Nunchuk app
#ifndef APPCONF_CHUK_CTRL_TYPE
#define APPCONF_CHUK_CTRL_TYPE				CHUK_CTRL_TYPE_CURRENT
#endif
#ifndef APPCONF_CHUK_HYST
#define APPCONF_CHUK_HYST					0.15
#endif
#ifndef APPCONF_CHUK_RAMP_TIME_POS
#define APPCONF_CHUK_RAMP_TIME_POS			0.9
#endif
#ifndef APPCONF_CHUK_RAMP_TIME_NEG
#define APPCONF_CHUK_RAMP_TIME_NEG			0.3
#endif
#ifndef APPCONF_STICK_ERPM_PER_S_IN_CC
#define APPCONF_STICK_ERPM_PER_S_IN_CC		3000.0
#endif
#ifndef APPCONF_CHUK_THROTTLE_EXP
#define APPCONF_CHUK_THROTTLE_EXP			0.0
#endif
#ifndef APPCONF_CHUK_THROTTLE_EXP_BRAKE
#define APPCONF_CHUK_THROTTLE_EXP_BRAKE		0.0
#endif
#ifndef APPCONF_CHUK_THROTTLE_EXP_MODE
#define APPCONF_CHUK_THROTTLE_EXP_MODE		THR_EXP_POLY
#endif
#ifndef APPCONF_CHUK_MULTI_ESC
#define APPCONF_CHUK_MULTI_ESC				false
#endif
#ifndef APPCONF_CHUK_TC
#define APPCONF_CHUK_TC						false
#endif
#ifndef APPCONF_CHUK_TC_MAX_DIFF
#define APPCONF_CHUK_TC_MAX_DIFF			3000.0
#endif

// NRF app
#ifndef APPCONF_NRF_SPEED
#define APPCONF_NRF_SPEED					NRF_SPEED_2M
#endif
#ifndef APPCONF_NRF_POWER
#define APPCONF_NRF_POWER					NRF_POWER_0DBM
#endif
#ifndef APPCONF_NRF_CRC
#define APPCONF_NRF_CRC						NRF_CRC_1B
#endif
#ifndef APPCONF_NRF_RETR_DELAY
#define APPCONF_NRF_RETR_DELAY				NRF_RETR_DELAY_250US
#endif
#ifndef APPCONF_NRF_RETRIES
#define APPCONF_NRF_RETRIES					3
#endif
#ifndef APPCONF_NRF_CHANNEL
#define APPCONF_NRF_CHANNEL					76
#endif
#ifndef APPCONF_NRF_ADDR_B0
#define APPCONF_NRF_ADDR_B0					0xC6
#endif
#ifndef APPCONF_NRF_ADDR_B1
#define APPCONF_NRF_ADDR_B1					0xC7
#endif
#ifndef APPCONF_NRF_ADDR_B2
#define APPCONF_NRF_ADDR_B2					0x0
#endif
#ifndef APPCONF_NRF_SEND_CRC_ACK
#define APPCONF_NRF_SEND_CRC_ACK			true
#endif

#endif
