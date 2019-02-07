/*
 * app_vescuino.c
 *
 *  Created on: 12 Feb, 2018
 *      Author: cdi
 */

#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "app.h"
#include "commands.h"
#include "chprintf.h"
#include "utils.h"
#include "shell.h"		//cdi, Add at makefile. "$(CHIBIOS)/os/various/shell.c \"
#include "stdio.h"
#include "stdlib.h"
#include "mcpwm_foc.h"
#include "app_util_cdi.c"
#include "app_vescuino.h"
#include "app_vescuino_interface.c"
#include "app_vescuino_spi.c"
#include "app_vescuino_mpu9250.c"
#include "app_smg_full_metal_control.c"
#include "app_diff_robot_control.c"

// Defines
#define SHELL_WA_SIZE   	THD_WORKING_AREA_SIZE(2048)

// Threads
static THD_FUNCTION(app_vescuino_thread, arg);
static THD_WORKING_AREA(app_vescuino_thread_wa, 4096);

// Private variables
static int tachometer_value_start;
static int tachometer_value_end;
static int polepair_num;
static bool tachometer_value_reset = true;

// --------------- Start of Shell Part ---------------
void debug_printf(const char *fmt, ...) {
#ifdef USE_SD2_DEBUG_PRINTF
	va_list ap;

	va_start(ap, fmt);
	chvprintf((BaseSequentialStream*) &SERIAL_DRIVER, fmt, ap);
	va_end(ap);
#else
	va_list arg;
	va_start (arg, fmt);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer+1, 254, fmt, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<254)? len+1: 255);
	}
#endif
}

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state           name time    \r\n");
  chprintf(chp, "-------------------------------------------------------------\r\n");
  tp = chRegFirstThread();
  do {

    chprintf(chp, "%08lx %08lx %4lu %4lu %9s %14s %lu\r\n",
             (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state], tp->p_name, (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void sys_reboot(void)
{
	// Lock the system and enter an infinite loop. The watchdog will reboot.
	debug_printf("\r\nrebooting...");
	chThdSleepMilliseconds(1000);
	__disable_irq();
	for(;;){};
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: rb\r\n");
		return;
	}

	sys_reboot();
}

static void cmd_spi_debug_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: sdp\r\n");
		return;
	}

	if(spi_debug_print==0) {
		spi_debug_print = 1;
		chprintf(chp, "spi_debug_print = 1\r\n");
	}
	else if(spi_debug_print==1) {
		spi_debug_print = 0;
		chprintf(chp, "spi_debug_print = 0\r\n");
	}
}

static void cmd_spi_devel_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: dev\r\n");
		return;
	}

	if(spi_devel_print==0) {
		spi_devel_print = 1;
		chprintf(chp, "spi_devel_print = 1\r\n");
	}
	else if(spi_devel_print==1) {
		spi_devel_print = 0;
		chprintf(chp, "spi_devel_print = 0\r\n");
	}
}

static void cmd_can_dev_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: lcd <list can devs>\r\n");
		return;
	}

	uint8_t id_num = 0;
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++)
	{
		can_status_msg *msg = comm_can_get_status_msg_index(i);

		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0)
		{
			chprintf(chp, "ID : %i / ", msg->id);
			chprintf(chp, "RX Time : %i / ", (int)msg->rx_time);
			chprintf(chp, "Age (msec) : %.2f\r\n", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
			id_num++;
		}
	}

	if(id_num==0)
	{
		chprintf(chp, "Can devices are not connected\r\n");
	}
}

static void cmd_spi_rx_read_dt_us_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: sdt <spi rx read dt print>\r\n");
		return;
	}

	chprintf(chp, "spi dt: %.3f msec, process error cnt:%d\r\n", (double)(spi_read_dt_us/1000.), spi_process_error_cnt);
}

static void cmd_spi_control_flag_toggle(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: sct <spi control enable or disable>\r\n");
		return;
	}

	if(spi_control_flag==0) {
		spi_control_flag = 1;
		spi_brake_cnt = 0;
		chprintf(chp, "spi_control_flag = 1\r\n");
	}
	else if(spi_control_flag==1) {
		spi_control_flag = 0;
		chprintf(chp, "spi_control_flag = 0, run current brake for 2 sec\r\n");
	}
}

static void cmd_get_polepair_num(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: gpn <print polepair number>\r\n");
		return;
	}

	if(tachometer_value_reset==true)
	{
		mc_interface_get_tachometer_value(true);
		chprintf(chp, "rotate motor for 1 revolution by hand\r\n");
		tachometer_value_start = mc_interface_get_tachometer_value(tachometer_value_reset);
		chprintf(chp, "tachometer value start:%d\r\n", tachometer_value_start);
		chprintf(chp, "After finishing rotation, run 'gpn' again\r\n");
		tachometer_value_reset = false;
	}
	else
	{
		tachometer_value_end = mc_interface_get_tachometer_value(tachometer_value_reset);
		if(tachometer_value_start==tachometer_value_end)	chprintf(chp, "error! the motor didn't rotated\r\n");
		else {
			polepair_num = abs(tachometer_value_end-tachometer_value_start)/6;
			chprintf(chp, "tachometer value end:%d\r\n", tachometer_value_end);
			chprintf(chp, "Pole Number:%d, Pole-pair Number:%d\r\n", polepair_num*2, polepair_num);
			chprintf(chp, "ERPM2RPM = %.4f\r\n", (double)(1./polepair_num));
			chprintf(chp, "TACHO2RAD = %.4f\r\n", (double)(1./(3.*polepair_num)));
			tachometer_value_reset = true;
			mc_interface_get_tachometer_value(true);
		}
	}
}

static void cmd_is_encoder_index_found(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: ind <print whether encoder index found>\r\n");
		return;
	}

	chprintf(chp, "sensor_mode <0=sensorless, 1=encoder, 2=hall>, encoder_offset:%.2f\r\n", (double)app_vescuino_dps_get_enc_offset());
	chprintf(chp, "sensor_mode:%d, enc_is_configured:%d, encoder_deg:%.2f deg, enc_index_found:%d\r\n", mcpwm_foc_check_sensor_mode(), encoder_is_configured(), (double)encoder_read_deg(), encoder_index_found());
}

static void cmd_imu_mpu9250_use_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_use <set mpu9250 use - 0:off, 1:on, 2:1st, 2nd on>\r\n");
		return;
	}

	int mode = -1;
	mode = atoi(argv[0]);

	if(mode>=0 && mode<=1)	app_mpu9250_set_use_mode(mode);
	if(mode==2)	app_mpu9250_set_use_mode(mode);
}

static void cmd_imu_mpu9250_param_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mpu_param <print mpu9250 parameter & update>\r\n");
		return;
	}

	app_mpu9250_param_printf();
}

void app_restore_default_setting(void)
{
	conf_general_get_default_custom_app_configuration(appconf);
	conf_general_store_app_configuration(appconf);
	debug_printf("\r\nCustom App Restore Default Setting");
	debug_printf("\r\n");
}

static void cmd_custom_app_default_param_restore(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: cdp <restore custom app default parameter>\r\n");
		return;
	}

	app_restore_default_setting();
}


static void cmd_imu_mpu9250_set_comm_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_comm 0 0 <set 1st, 2nd mpu9250 comm mode - 0:spi, 1:i2c>\r\n");
		return;
	}

	if(argc==2)
	{
		int mode1, mode2;
		mode1 = atoi(argv[0]);
		mode2 = atoi(argv[1]);

		if(mode1>=0 && mode1<=1) app_mpu9250_set_comm_mode(mode1, 0);
		if(mode2>=0 && mode2<=1) app_mpu9250_set_comm_mode(mode2, 1);
	}
	else
	{
		chprintf(chp, "Usage: mpu_comm 0 0 <set 1st, 2nd mpu9250 comm mode - 0:spi, 1:i2c>\r\n");
	}
}

static void cmd_imu_mpu9250_set_setup_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_setup 0 0 <set 1st, 2nd mpu9250 setup mode - 0:basic, 1:fast, 2:full>\r\n");
		return;
	}

	if(argc==2)
	{
		int mode1, mode2;
		mode1 = atoi(argv[0]);
		mode2 = atoi(argv[1]);

		if(mode1>=0 && mode1<=2) app_mpu9250_set_imu_setup_mode(mode1, 0);
		if(mode2>=0 && mode2<=2) app_mpu9250_set_imu_setup_mode(mode2, 1);
	}
	else
	{
		chprintf(chp, "Usage: mpu_setup 0 0 <set 1st, 2nd mpu9250 setup mode - 0:basic, 1:fast, 2:full>\r\n");
	}
}

static void cmd_imu_mpu9250_set_dof_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_dof 0 <set 1st, 2nd mpu9250 setup mode - 0:6dof, 1:9dof>\r\n");
		return;
	}

	if(argc==2)
	{
		int mode1, mode2;
		mode1 = atoi(argv[0]);
		mode2 = atoi(argv[1]);

		if(mode1>=0 && mode1<=1) app_mpu9250_set_imu_dof_mode(mode1, 0);
		if(mode2>=0 && mode2<=1) app_mpu9250_set_imu_dof_mode(mode2, 1);
	}
	else
	{
		chprintf(chp, "Usage: mpu_dof 0 <set 1st, 2nd mpu9250 setup mode - 0:6dof, 1:9dof>\r\n");
	}
}

static void cmd_imu_mpu9250_set_acc_user_bias(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_acc_bias 0 1.0 2.0 3.0 <set mpu9250 acc bias user defined (mg)>\r\n");
		return;
	}

	static uint8_t ch;
	static float bx, by, bz;
	if(argc>0) ch = atoi(argv[0]);
	if(argc>1) bx = atof(argv[1]);
	if(argc>2) by = atof(argv[2]);
	if(argc>3) bz = atof(argv[3]);

	if(argc==4) app_mpu9250_set_imu_acc_user_bias(bx/1000., by/1000., bz/1000., ch);	// store mg to g
	else chprintf(chp, "Usage: mpu_acc_bias <ch_num> 1.0 2.0 3.0 (<ch_num>:0 or 1)\r\n");
}

static void cmd_imu_mpu9250_set_acc_user_bias_onoff(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: mpu_aub_use 0 0 <set 1st, 2nd mpu9250 acc user bias - 0:OFF, 1:ON>\r\n");
		return;
	}

	if(argc==2)
	{
		int mode1, mode2;
		mode1 = atoi(argv[0]);
		mode2 = atoi(argv[1]);

		if(mode1>=0 && mode1<=1) app_mpu9250_set_imu_acc_user_bias_onoff(mode1, 0);
		if(mode2>=0 && mode2<=1) app_mpu9250_set_imu_acc_user_bias_onoff(mode2, 1);
	}
	else
	{
		chprintf(chp, "Usage: mpu_aub_use 0 0 <set 1st, 2nd mpu9250 acc user bias - 0:OFF, 1:ON>\r\n");
	}
}

static void cmd_smg_set_lqr_gain(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: slg q a b c <set smg lqr gain q:axis(x:0,y:1,z:2), a:qr_1, b:lqr_2, c:lqr_3>\r\n");
		return;
	}

	uint8_t axis = 0;
	static float lqr_1, lqr_2, lqr_3;
	if(argc>0) axis = atoi(argv[0]);
	if(argc>1) lqr_1 = atof(argv[1]);
	if(argc>2) lqr_2 = atof(argv[2]);
	if(argc>3) lqr_3 = atof(argv[3]);

	if((argc==4) && (axis<3)) {
		app_smg_set_lqr_gain(lqr_1, lqr_2, lqr_3, axis);	// store mg to g
	}
	else chprintf(chp, "Usage: slg q a b c <set smg lqr gain q:axis(x:0,y:1,z:2), a:lqr_1, b:lqr_2, c:lqr_3>\r\n");
}

static void cmd_imu_mpu9250_debug_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;

	static bool flag_mdp = 0;
	static int mode_mdp = 2;

	if (mode_mdp < 0) {
		chprintf(chp, "Usage: mdp <mpu9250 debug print>\r\n");
		chprintf(chp, "Usage: mdp 0 <mpu9250 debug print - RAW digit>\r\n");
		chprintf(chp, "Usage: mdp 1 <mpu9250 debug print - RAW value>\r\n");
		chprintf(chp, "Usage: mdp 2 <mpu9250 debug print - RPY deg>\r\n");
		chprintf(chp, "Usage: mdp 3 <mpu9250 debug print - RPY Gyro Rate>\r\n");
		chprintf(chp, "Usage: mdp 4 <mpu9250 debug print - RPY deg - SMG Relative>\r\n");
		return;
	}
	else
	{
		if(argc==1) {
			flag_mdp = 1;
			mode_mdp = atoi(argv[0]);
		}
		else {
			if(flag_mdp==0) flag_mdp = 1;
			else 			flag_mdp = 0;
		}
	}

	app_mpu9250_set_debug_print(flag_mdp, mode_mdp);
}

static void cmd_set_custom_app_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc <= 0) {
		chprintf(chp, "Usage: sca 0 <set custom app - 0:VESCuino SPI, 1:SMG>\r\n");
		return;
	}

	int mode = -1;
	mode = atoi(argv[0]);

	if(mode==0)	{
		appconf->app_custom.custom_app_mode = mode;
		conf_general_store_app_configuration(appconf);
		chprintf(chp, "Set Custom App : VESCuino SPI\r\n");
	}
	else if(mode==1) {
		appconf->app_custom.custom_app_mode = mode;
		conf_general_store_app_configuration(appconf);
		chprintf(chp, "Set Custom App : SMG\r\n");
	}
	else {
		chprintf(chp, "error, select 0 or 1\r\n");
	}
}

static void cmd_smg_param_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;

	int mode_smg = 0;	// default value: periodic status print

	if (argc == 0) {
		chprintf(chp, "Usage: smg 0 <SMG periodic print off>\r\n");
		chprintf(chp, "Usage: smg 1 <SMG periodic print on>\r\n");
		chprintf(chp, "Usage: smg 2 <SMG parameter print>\r\n");
		return;
	}
	else
	{
		if(argc==1) {
			mode_smg = atoi(argv[0]);
			app_smg_print_status(mode_smg);
		}
		else {
			mode_smg = 0;
			chprintf(chp, "Usage: smg 0 <SMG periodic print off>\r\n");
			chprintf(chp, "Usage: smg 1 <SMG periodic print on>\r\n");
			chprintf(chp, "Usage: smg 2 <SMG parameter print>\r\n");
		}
	}
}

static void cmd_smg_nrf_values_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: nrf <print nrf normalized values, every 1sec>\r\n");
		return;
	}

	static int flag_nrf = 0;

	if(flag_nrf==0) {
		flag_nrf = 1;
	}
	else if(flag_nrf==1) {
		flag_nrf = 0;
	}
	app_smg_nrf_values_print(flag_nrf);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"rb", cmd_reboot},
  {"gpn", cmd_get_polepair_num},
  {"ind", cmd_is_encoder_index_found},
  {"lcd", cmd_can_dev_print},
  {"dev", cmd_spi_devel_print},
  {"sdp", cmd_spi_debug_print},
  {"sdt", cmd_spi_rx_read_dt_us_print},
  {"sct", cmd_spi_control_flag_toggle},
  {"cdp", cmd_custom_app_default_param_restore},
  {"mpu_use", cmd_imu_mpu9250_use_mode},
  {"mpu_param", cmd_imu_mpu9250_param_print},
  {"mpu_comm", cmd_imu_mpu9250_set_comm_mode},
  {"mpu_setup", cmd_imu_mpu9250_set_setup_mode},
  {"mpu_dof", cmd_imu_mpu9250_set_dof_mode},
  {"mpu_aub", cmd_imu_mpu9250_set_acc_user_bias},
  {"mpu_aub_use", cmd_imu_mpu9250_set_acc_user_bias_onoff},
  {"mdp", cmd_imu_mpu9250_debug_print},
  {"sca", cmd_set_custom_app_mode},
  {"slg", cmd_smg_set_lqr_gain},
  {"smg", cmd_smg_param_print},
  {"nrf", cmd_smg_nrf_values_print},
  {NULL, NULL}
};

// UART SERIAL_DRIVER(Shell) Baudrate - SERIAL_DEFAULT_BITRATE @halconf.h
static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *) &SERIAL_DRIVER,
  commands
};
// --------------- End of Shell Part ---------------

// --------------- App Custom Part ---------------
void app_custom_start(void)
{
	// Start USART Driver
	// modifying required at mcuconf.h and halconf.h for using Serial Driver
	// Baudrate Setting -> #define SERIAL_DEFAULT_BITRATE @halconf.h
#ifdef USE_VESCUINO_PCB
	sdStart(&SERIAL_DRIVER, NULL);
	palSetPadMode(HW_UART2_TX_PORT, HW_UART2_TX_PIN, PAL_MODE_ALTERNATE(HW_UART2_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART2_RX_PORT, HW_UART2_RX_PIN, PAL_MODE_ALTERNATE(HW_UART2_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
#endif

	// Stop other functions
	hw_stop_i2c();
	app_nunchuk_stop();
	app_ppm_stop();
	app_adc_stop();
	app_uartcomm_stop();

	// Create VESCuino Thread
	stop_now = false;
	chThdCreateStatic(app_vescuino_thread_wa, sizeof(app_vescuino_thread_wa), NORMALPRIO, app_vescuino_thread, NULL);
}

void app_custom_stop(void)
{
	stop_now = true;

	if (is_running) {
		spiStop(&HW_SPI_DEV);
		is_running = 0;
	}
}

void app_custom_configure(app_configuration *conf)
{
	appconf = conf;
}

// Misc.
void app_wait_sec(float second)
{
	uint32_t millisecond;
	millisecond = (uint32_t)(1000*second);
	chThdSleepMilliseconds(millisecond);
}

void app_wait_ms(float msec)
{
	chThdSleepMilliseconds(msec);
}

uint32_t app_read_us(void)
{
	systime_t time = chVTGetSystemTime();
	return ST2US(time);
}

uint32_t app_read_ms(void)
{
	systime_t time = chVTGetSystemTime();
	return ST2MS(time);
}

//
bool app_run_every_x_hz(double x_hz, uint32_t cnt, double hz)
{
	bool run;

	static uint16_t target_cnt;

	target_cnt = (uint16_t)(hz/x_hz);

	if(cnt%target_cnt==0) run = true;
	else run = false;

	return run;
}
// --------------- End of App Custom Part ---------------

/*
 * VESCuino Thread
 */
static THD_FUNCTION(app_vescuino_thread, arg) {
	(void)arg;

	chRegSetThreadName("VESCuino Main");
	chThdSleep(10);
	debug_printf("\r\nStart VESCuino Shell Thread");
	is_running = true;

	/*
	* Shell manager initialization.
	*/
	thread_t *shelltp = NULL;
	shellInit();

	if(appconf->app_custom.custom_app_mode==CUSTOM_APP_VESCuino)	// VESCuino SPI
	{
		// Initialize packet handler
		packet_init(send_packet_spi_slave, process_packet_spi_slave, PACKET_HANDLER);
		//commands_set_send_func(send_packet_wrapper_spi_slave);

		// custom app
		commands_set_app_data_handler(send_custom_app_data);

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
		// SPI interface
		spi1_peripheral_setting_slave();
		// SPI Start
		spiStart(&HW_SPI_DEV, &spicfg);
		// SPI Start Slave Threads
		chThdCreateStatic(spi_slave_read_thread_wa, sizeof(spi_slave_read_thread_wa), NORMALPRIO, spi_slave_read_thread, NULL);
		debug_printf("\r\nVESCuino SPI RX Thread Start");
#endif

		// debug thread
		chThdCreateStatic(custom_cmd_debug_thread_wa, sizeof(custom_cmd_debug_thread_wa), NORMALPRIO, custom_cmd_debug_thread, NULL);
		// 10ms thread
		chThdCreateStatic(custom_cmd_check_thread_wa, sizeof(custom_cmd_check_thread_wa), NORMALPRIO, custom_cmd_check_thread, NULL);

		// COMM_SET_DPS Thread
		app_vescuino_set_dps_thread_start();
	}
	else if(appconf->app_custom.custom_app_mode==CUSTOM_APP_SMG_FULL_METAL)	// SMG Mode
	{
		debug_printf("\r\nStart SMG Control Thread");
		app_smg_full_metal_thread_start();
	}
	else if(appconf->app_custom.custom_app_mode==CUSTOM_APP_DIFF_ROBOT)	// Differential Robot Mode
	{
		debug_printf("\r\nStart Differential Robot Control Thread");
		app_diff_robot_thread_start();
	}

	// MPU9250 Start
	if(!app_mpu9250_start_flag) {
		app_mpu9250_param_update();
		if(imu_data.use_mode>=1)	app_mpu9250_start();
		app_mpu9250_start_flag = true;
	}

	// Loop
	for(;;) {
		time_main = chVTGetSystemTime();

		// Make Shell
		if (!shelltp && (SERIAL_DRIVER.state == SD_READY))
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminatedX(shelltp)) {
			chThdRelease(shelltp);	// Recovers memory of the previous shell.
			shelltp = NULL;			// Triggers spawning of a new shell.
		}

		if(appconf->app_custom.custom_app_mode==CUSTOM_APP_VESCuino)	// VESCuino SPI
		{
			// Arduino SPI Connection Check
			if(spi_process_error_cnt == 0 && spi_host_get_connected_flag == 0 && spi_host_model!=0) {
				if(spi_host_model==ARDUINO_MEGA)			debug_printf("\r\nSPI Connected, Master:Arduino_Mega_2560, Freq:%.2f Hz", (double)(1000000./spi_read_dt_us));
				else if(spi_host_model==ARDUINO_DUE)		debug_printf("\r\nSPI Connected, Master:Arduino_DUE, Freq:%.2f Hz", (double)(1000000./spi_read_dt_us));
				else if(spi_host_model==ARDUINO_TEENSY_32)	debug_printf("\r\nSPI Connected, Master:Arduino_Teensy_3.2, Freq:%.2f Hz", (double)(1000000./spi_read_dt_us));
				else if(spi_host_model==ARDUINO_TEENSY_36)	debug_printf("\r\nSPI Connected, Master:Arduino_Teensy_3.6, Freq:%.2f Hz", (double)(1000000./spi_read_dt_us));
				else if(spi_host_model==USB)				debug_printf("\r\nUSB Connected, %.2f sec", (double)(ST2MS(time_main)/1000.));
				else 										debug_printf("\r\nSPI Connected, Master:Unknown");
				spi_control_flag = 1;
				spi_host_get_connected_flag = 1;
				spi_read_dt_freq = (int)(1000000./spi_read_dt_us);
			}

			// COMM_SET detection
			if(spi_host_get_connected_flag==1 && spi_host_model!=USB)
			{
				int p = 0;
				char str[20];
				for(int i=0; i<spi_num_of_id; i++) {
					if(i==0) p += sprintf(str + p, "%d(1) ", spi_comm_set_index[i]);
					else p += sprintf(str + p, "%d(%d) ", spi_comm_set_index[i], active_can_devs[i]);
					spi_comm_set_index[i] = 0;
				}
				debug_printf("\r\n[%.3f sec] spi dt: %.3f msec, err_cnt: %d, cont_mode(en): %s", (double)(ST2MS(time_sys)/1000.), (double)(spi_read_dt_us/1000.), spi_process_error_cnt, str);
			}
		}

		chThdSleepMilliseconds(1000);
	}
}
