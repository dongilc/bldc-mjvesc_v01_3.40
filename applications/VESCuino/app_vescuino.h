/*
 * app_vescuino.h
 *
 *  Created on: 2018. 2. 24.
 *      Author: cdi
 */

#ifndef APPLICATIONS_APP_VESCUINO_H_
#define APPLICATIONS_APP_VESCUINO_H_

#ifdef USE_SERIAL_DRIVER_UART3
#define SERIAL_DRIVER	SD3
#else
#define SERIAL_DRIVER	SD2
#endif

//
enum CUSTOM_APP_HOST_TYPE {
	UNKNOWN = 0,
	ARDUINO_MEGA,
	ARDUINO_DUE,
	ARDUINO_TEENSY_32,
	ARDUINO_TEENSY_36,
	USB
};

// Global variable
static volatile bool is_running = false;
static volatile bool stop_now = true;
static app_configuration *appconf;

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
static thread_t *spi_read_tp = NULL;
#endif

// @ app_vescuino.c
void debug_printf(const char *fmt, ...);
void app_wait_sec(float second);
void app_wait_ms(float msec);
uint32_t app_read_us(void);
uint32_t app_read_ms(void);
bool app_run_every_x_hz(double x_hz, uint32_t cnt, double hz);
static void sys_reboot(void);
void app_restore_default_setting(void);

// @ app_vescuino_mpu9250.c
void app_mpu9250_param_update(void);
void app_mpu9250_set_debug_print(bool print_flag, uint8_t print_mode);
void app_mpu9250_param_printf(void);
void app_mpu9250_set_comm_mode(uint8_t mode, uint8_t ch);
void app_mpu9250_set_imu_setup_mode(uint8_t mode, uint8_t ch);
void app_mpu9250_set_imu_dof_mode(uint8_t mode, uint8_t ch);
void app_mpu9250_set_imu_gyro_bias(float bias_x, float bias_y, float bias_z, uint8_t ch);
void app_mpu9250_set_imu_acc_bias(float bias_x, float bias_y, float bias_z, uint8_t ch);
void app_mpu9250_set_imu_acc_user_bias(float bias_x, float bias_y, float bias_z, uint8_t ch);
void app_mpu9250_set_imu_acc_user_bias_onoff(uint8_t mode, uint8_t ch);

#endif /* APPLICATIONS_APP_VESCUINO_H_ */
