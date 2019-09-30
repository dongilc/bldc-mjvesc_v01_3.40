/*
 * app_vescuino_spi.c
 *
 *  Created on: 20 March, 2018
 *      Author: cdi
 */

#include "app_vescuino_spi.h"
#include "mc_interface.h"
#include "timeout.h"
#include "comm_can.h"
#include "isr_vector_table.h"
#include "buffer.h"
#include "utils.h"
#include "encoder.h"
#include "app_vescuino_dps_control.c"

/*
 * SPI1 Peripheral Setting
 */
void spi1_peripheral_setting_slave(void)
{
	/*
	 * SPI1 I/O pins setup.
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// SPI pin
	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));     /* SCK.     */
	palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));// | PAL_STM32_OSPEED_HIGHEST);   /* MISO.    */
	palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));   /* MOSI.    */
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_INPUT_PULLUP);

#ifdef USE_VESCUINO_PCB
	// Slave RX Ready - Output
	palSetPadMode(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD, PAL_MODE_OUTPUT_PUSHPULL);// | PAL_STM32_OSPEED_HIGHEST); //PAL_STM32_OTYPE_OPENDRAIN
	palClearPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD);

	// Arduino Reset Pin - Output, Default High
	palSetPadMode(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET);
#endif

#ifdef USE_DEBUG_LED_TOGGLE
	// Debug Pin - Output
	palSetPadMode(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);// | PAL_STM32_OSPEED_HIGHEST); //PAL_STM32_OTYPE_OPENDRAIN
	palClearPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG);
#endif
}

void saveDataToBuffer(unsigned char *data, int len)
{
	for(int i=0; i<len; i++)
	{
		spi_rx_buffer[spi_rx_write_pos++] = data[i];

		if (spi_rx_write_pos >= SPI_BUFFER_SIZE) {
				spi_rx_write_pos = 0;
		}
	}
}

void loadDataProcessPacket(void)
{
	while (spi_rx_read_pos != spi_rx_write_pos) {
		packet_process_byte(spi_rx_buffer[spi_rx_read_pos++], PACKET_HANDLER);	// process_packet_spi -> and then -> send_packet_spi

		if (spi_rx_read_pos == SPI_BUFFER_SIZE) {
			spi_rx_read_pos = 0;
		}
	}
#ifdef USE_DEBUG_LED_TOGGLE
	if(spi_debug_led_cnt>=spi_read_dt_freq) {
		SPI_DEBUG_LED_TOGGLE();
		spi_debug_led_cnt = 0;
	}
	spi_debug_led_cnt++;
#endif
}

uint8_t get_number_of_can_status(void)
{
	uint8_t id_num = 0;
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++)
	{
		can_status_msg *msg = comm_can_get_status_msg_index(i);

		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) id_num++;
	}

	return id_num;
}

/*
 * Send COMM* commands reply - slave
 */
void comm_reply_slave_spi(unsigned char *packet_data, unsigned int packet_len)
{
	for(unsigned int i=0; i<SPI_FIXED_DATA_BYTE; i++) {
		if(i<packet_len)		spi_tx_bytes[i] = packet_data[i];
		else					spi_tx_bytes[i] = 0;
	}
}

static void send_packet_spi_slave(unsigned char *data, unsigned int len)
{
	if(spi_debug_print==1) {
		debug_printf("ready to reply comm\r\n - len:%d, \r\n - data:");
		for(unsigned int i=0; i<len; i++)
		{
			if(i%10==0)	debug_printf("\r\n");
			debug_printf("%d ", data[i]);
		}
		debug_printf("\r\n");
	}

	comm_reply_slave_spi(data, len);
}

static void send_packet_wrapper_spi_slave(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void process_packet_spi_slave(unsigned char *data, unsigned int len) {
	// Packet Receiving Part
	commands_set_send_func(send_packet_wrapper_spi_slave);
	commands_process_packet(data, len);
}

static void send_custom_app_data(unsigned char *data, unsigned int len) {
	(void)len;

	// Rx Part
	int32_t ind = 0;
	uint8_t num_of_id = 0;

	if(spi_debug_print==1) debug_printf("custom rx done. len=%d\r\n", len);

	spi_host_model = data[ind++];

	if(spi_host_model!=UNKNOWN)
	{
		spi_num_of_id = num_of_id = data[ind++];
		for(int i=0; i<num_of_id; i++)
		{
			id_set[i] = data[ind++];
			comm_set[i] = data[ind++];

			if(id_set[i]==0)
			{
				switch(comm_set[i]) {
					case COMM_SET_DUTY:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 100000.0;
						mc_interface_set_duty(value_set[i]);
						timeout_reset();
						spi_comm_set_index[i] = COMM_SET_DUTY;
						break;
					case COMM_SET_CURRENT:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						mc_interface_set_current(value_set[i]);
						timeout_reset();
						spi_comm_set_index[i] = COMM_SET_CURRENT;
						break;
					case COMM_SET_CURRENT_BRAKE:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						mc_interface_set_brake_current(value_set[i]);
						timeout_reset();
						spi_comm_set_index[i] = COMM_SET_CURRENT_BRAKE;
						break;
					case COMM_SET_RPM:
						value_set[i] = (float)buffer_get_int32(data, &ind);
						mc_interface_set_pid_speed(value_set[i]);
						timeout_reset();
						spi_comm_set_index[i] = COMM_SET_RPM;
						break;
					case COMM_SET_POS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000000.0;
						mc_interface_set_pid_pos(value_set[i]);
						timeout_reset();
						spi_comm_set_index[i] = COMM_SET_POS;
						break;
					case COMM_SET_DPS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_vescuino_set_dps(value_set[i]);
						spi_comm_set_index[i] = COMM_SET_DPS;
						break;
					case COMM_SET_GOTO:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0 ;
						app_vescuino_set_goto(value_set[i]);
						spi_comm_set_index[i] = COMM_SET_GOTO;
						break;
					default:
						spi_comm_set_index[i] = -1;	//error
						break;
				}
			}
			else
			{
				// CAN DEVs
				can_status_msg *msg = comm_can_get_status_msg_id(id_set[i]);

				switch(comm_set[i]) {
					case COMM_SET_DUTY:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 100000.0;
						spi_comm_set_index[i] = COMM_SET_DUTY;
						if(msg!=0) comm_can_set_duty(id_set[i], value_set[i]);
						break;
					case COMM_SET_CURRENT:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						spi_comm_set_index[i] = COMM_SET_CURRENT;
						if(msg!=0) comm_can_set_current(id_set[i], value_set[i]);
						break;
					case COMM_SET_CURRENT_BRAKE:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						spi_comm_set_index[i] = COMM_SET_CURRENT_BRAKE;
						if(msg!=0) comm_can_set_current_brake(id_set[i], value_set[i]);
						break;
					case COMM_SET_RPM:
						value_set[i] = (float)buffer_get_int32(data, &ind);
						spi_comm_set_index[i] = COMM_SET_RPM;
						if(msg!=0) comm_can_set_rpm(id_set[i], value_set[i]);
						break;
					case COMM_SET_POS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000000.0;
						spi_comm_set_index[i] = COMM_SET_POS;
						if(msg!=0) comm_can_set_pos(id_set[i], value_set[i]);
						break;
					case COMM_SET_DPS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 100.0;
						app_vescuino_set_dps(value_set[i]);
						spi_comm_set_index[i] = COMM_SET_DPS;
						if(msg!=0) comm_can_set_dps(id_set[i], value_set[i]);
						break;
					default:
						spi_comm_set_index[i] = -1;	//error
						break;
				}
			}

			if(spi_debug_print==1) debug_printf("custom set! i=%d, id=%d, comm_set=%d, value=%.4f\r\n", i, id_set[i], comm_set[i], (double)value_set[i]);
		}
	}

	// Tx Part
	ind = 0;
	uint8_t send_buffer[SPI_FIXED_DATA_BYTE] = {0,};
	send_buffer[ind++] = COMM_CUSTOM_APP_DATA;	// +1

	// get value return
	spi_send_mode = 80;
	send_buffer[ind++] = spi_send_mode; // +1
	// fw
	send_buffer[ind++] = FW_VERSION_MAJOR;	// +1
	send_buffer[ind++] = FW_VERSION_MINOR;	// +1
	// get value
	buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
	buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
	buffer_append_float32(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind);
	buffer_append_float32(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind);
	buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
	buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind);
	buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
	buffer_append_float32(send_buffer, mc_interface_get_amp_hours(false), 1e4, &ind);
	buffer_append_float32(send_buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);
	buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
	buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
	buffer_append_int32(send_buffer, mc_interface_get_tachometer_value(false), &ind);
	buffer_append_int32(send_buffer, mc_interface_get_tachometer_abs_value(false), &ind);
	send_buffer[ind++] = mc_interface_get_fault();
	buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
	//cdi
	buffer_append_float32(send_buffer, encoder_read_rps(), 1e5, &ind);
	buffer_append_float32(send_buffer, encoder_read_rad(), 1e2, &ind);

	// can status msg
	spi_send_mode = 81;
	uint8_t can_devs_num = 0;
	can_devs_num = get_number_of_can_status();
	send_buffer[ind++] = spi_send_mode;	// +1
	send_buffer[ind++] = can_devs_num;	// +1
	// can_status_msgs
	for(int i=0; i<can_devs_num; i++)
	{
		can_st_msg = comm_can_get_status_msg_id(i+1);	// +1
		if(can_st_msg!=0) // +11 byte
		{
			send_buffer[ind++] = can_st_msg->id;
			//buffer_append_float32(send_buffer, can_st_msg->rpm, 1e0, &ind);
			//buffer_append_float32(send_buffer, can_st_msg->current, 1e2, &ind);	// original can_status_msg
			//buffer_append_float16(send_buffer, can_st_msg->duty, 1e3, &ind);		// original can_status_msg
			//buffer_append_int32(send_buffer, can_st_msg->tachometer, &ind);		// modified can_status_msg
			buffer_append_float32(send_buffer, can_st_msg->rps, 1e5, &ind);		// modified can_status_msg
			buffer_append_float32(send_buffer, can_st_msg->rad, 1e2, &ind);		// modified can_status_msg
			active_can_devs[can_st_msg->id] = 1;
		}
		else	active_can_devs[can_st_msg->id] = 0;
	}
	commands_send_packet(send_buffer, ind);
	if(spi_debug_print==1) debug_printf("spi return value send complete\r\n");

	custom_app_data_process_cnt++;
	spi_process_flag = 0;
	spi_process_error_cnt = 0;
	spi_restart_flag = 0;
}

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
// SPI RX Thread
static THD_FUNCTION(spi_slave_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("spi1 slave rx");

	time_sys = chVTGetSystemTime();
	spi_read_tp = chThdGetSelfX();

	chThdSleepMilliseconds(1000);

	// can devs encoder reset
	for(int i=0; i<VESCUINO_STACK_NUM_MAX; i++) {
		comm_can_set_encoder_reset(i);
	}

	for(;;) {
		systime_t time_elapsed;
		time_elapsed = chVTTimeElapsedSinceX(time_sys);
		time_sys = chVTGetSystemTime();
		spi_read_dt_us = ST2US(time_elapsed);

		if (stop_now) {
			is_running = false;
			return;
		}

		// SPI data exchange
#ifdef USE_VESCUINO_PCB
		SPI_Rx_RDY_SET_LOW();
#endif
		while(spi_start_flag==0 && SPI_CS_READ()) {
			chThdSleepMilliseconds(10);
		}
		spi_start_flag = 1;
		spiExchange(&HW_SPI_DEV, SPI_FIXED_DATA_BYTE, spi_tx_bytes, spi_rx_bytes);

		if(spi_debug_print==1)	{
			debug_printf("--------------------------------------------------------\r\n");
			debug_printf("exchange done, time=%d, dt_us=%d\r\n rx_data: ", time_sys, spi_read_dt_us);
			for(int i=0; i<SPI_FIXED_DATA_BYTE; i++) debug_printf("%d ", spi_rx_bytes[i]);
			debug_printf("\r\n");
		}
		if(spi_process_flag==1) {
			if(spi_debug_print==1) debug_printf("RX Packet Error\r\n");
			spi_process_error_cnt++;

			// restart for host reconnection when error occur
			if(spi_process_error_cnt>=SPI_ERROR_CNT_FOR_REBOOT) {
				// reboot
				sys_reboot();
			}
		}
		else {
			// No error
#ifdef USE_VESCUINO_PCB
			SPI_Rx_RDY_SET_HIGH();
#endif
			spi_process_error_cnt = 0;
		}

		spi_process_flag = 1;
		spi_communication_cnt++;
		saveDataToBuffer(spi_rx_bytes, SPI_FIXED_DATA_BYTE);

		if(spi_control_flag)
		{
			loadDataProcessPacket();
		}
		else
		{
			if(spi_brake_cnt<=20)
			{
				// current brake for 2 sec
				float curr = 2.0;

				mc_interface_set_brake_current(curr);
				timeout_reset();	// needed after every 'mc_interface_set_current()'
				// can devs
				for(int i=0; i<get_number_of_can_status(); i++)	comm_can_set_current_brake(i+1, curr);

				spi_brake_cnt++;
			}

			spi_process_flag = 0;
			spi_process_error_cnt = 0;

			chThdSleepMilliseconds(10);
		}
	}
}
#endif

// custom cmd debug thread
static THD_FUNCTION(custom_cmd_debug_thread, arg) {
	(void)arg;

	chRegSetThreadName("custom_cmd debug");
	chThdSleepMilliseconds(2000);

	for(;;) {
		if(spi_devel_print==1)
		{
			debug_printf("\r\n--------------------------------------------------------");
			debug_printf("\r\n 0 - enc_rps:%f, enc_rad:%f, value_set:%f", (double)encoder_read_rps(), (double)encoder_read_rad(), (double)value_set[0]);
			for(int i=0; i<VESCUINO_STACK_NUM_MAX; i++)
			{
				if(active_can_devs[i]==1)
				{
					can_st_msg = comm_can_get_status_msg_id(i);
					debug_printf("\r\n %d - enc_rps:%f, enc_rad:%f, value_set:%f", i, (double)can_st_msg->rps, (double)can_st_msg->rad, (double)value_set[1]);
				}
			}
		}

		//debug_printf("\r\nspi_communication_cnt:%d", spi_communication_cnt);

		chThdSleepMilliseconds(1000);
	}
}

// Custom_cmd Connection Check thread
static THD_FUNCTION(custom_cmd_check_thread, arg) {
	(void)arg;

	chRegSetThreadName("custom_cmd conn. check");
	chThdSleepMilliseconds(2000);

	for(;;) {
		// Arduino Disconnection Check
		if(spi_host_get_connected_flag==1 && spi_communication_cnt == 0 && spi_host_model!=USB) {
			debug_printf("\r\nSPI Master Disconnected\r\n");
			spi_process_error_cnt++;
			spi_host_get_connected_flag = 0;

			//mc_interface_set_brake_current(5.0);
			//timeout_reset();	// needed after every 'mc_interface_set_current()'
			// can devs
			//for(int i=0; i<get_number_of_can_status(); i++)	comm_can_set_current_brake(i+1, 5.0);
		}

		// USB Connection Check
		if(custom_app_data_process_cnt==0 && spi_host_model==USB) {
			debug_printf("\r\nUSB Disconnected, %.2f sec\r\n", (double)(ST2MS(time_main)/1000.));
			spi_host_get_connected_flag = 0;
			spi_host_model = 0;
		}

		spi_communication_cnt = 0;
		custom_app_data_process_cnt = 0;

		chThdSleepMilliseconds(100);
	}
}
