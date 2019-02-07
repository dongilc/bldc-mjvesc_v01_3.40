/*
 * app_vescuino_spi.h
 *
 *  Created on: 20 March, 2018
 *      Author: cdi
 */

#ifndef APPLICATIONS_APP_VESCUINO_SPI_H_
#define APPLICATIONS_APP_VESCUINO_SPI_H_

#include "packet.h"
#include "commands.h"
#include "app_vescuino.h"
#include "app_vescuino_interface.h"

#define SPI_BUFFER_SIZE					1024
#define PACKET_HANDLER					2		// 0:USB, 1:UART, 2:SPI - increase PACKET_HANDLERS as 3 @packet.h
#define SPI_FIXED_DATA_BYTE				128
#define VESCUINO_STACK_NUM_MAX			10
#define SPI_ERROR_CNT_FOR_REBOOT		5

// Read Chip Select
#define SPI_CS_READ()				palReadPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS)

//
#ifdef HW_VERSION_60
// SPI READY = GPIOC5 (ADC3)
#define HW_SPI_PORT_SRXRD		GPIOC
#define HW_SPI_PIN_SRXRD		5
#endif

//
#ifdef USE_VESCUINO_PCB
#define SPI_Rx_RDY_SET_LOW()		palClearPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD)
#define SPI_Rx_RDY_SET_HIGH()		palSetPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD)
#define SPI_DEBUG_LED_ON()			palClearPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_DEBUG_LED_OFF()			palSetPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_DEBUG_LED_TOGGLE()		palTogglePad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_ARDUINO_nRESET_LOW()	palClearPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET)
#define SPI_ARDUINO_nRESET_HIGH()	palSetPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET)
#endif

/*
 * SPI1 Baudrate Setting
 */
#define SPI_SPEED_SETTING_REG	0		//Speed 42MHz. Doesn't matter at spi_slave

/*
 * SPI1 configuration structure.
 * common setting : CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 */
static const SPIConfig spicfg = {SPI_SLAVE, NULL, HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, SPI_SPEED_SETTING_REG};

// Sharing variables in vescuino-spi
static int spi_communication_cnt = 0;
static int spi_restart_flag = 0;
static int custom_app_data_process_cnt = 0;
static uint8_t spi_process_flag = 0;
static uint8_t spi_process_error_cnt = 0;
static uint8_t spi_control_flag = 1;
static uint8_t spi_brake_cnt = 0;
static uint8_t spi_send_mode = 80;
static uint8_t spi_host_model = 0;
static uint8_t spi_num_of_id = 0;
static int spi_comm_set_index[VESCUINO_STACK_NUM_MAX];
static bool spi_host_get_connected_flag = 0;
static int spi_rx_read_pos = 0;
static int spi_rx_write_pos = 0;
static unsigned char spi_rx_buffer[SPI_BUFFER_SIZE];
static unsigned char spi_tx_bytes[SPI_FIXED_DATA_BYTE];
#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
static unsigned char spi_rx_bytes[SPI_FIXED_DATA_BYTE];
static uint8_t spi_start_flag = 0;
#endif
static systime_t time_main;
static systime_t time_sys;
static int spi_read_dt_us;
static int spi_read_dt_freq;
static uint8_t spi_debug_print = 0;
static uint8_t spi_devel_print = 0;
#ifdef USE_DEBUG_LED_TOGGLE
static int spi_debug_led_cnt = 0;
#endif

// Received data
static uint8_t active_can_devs[VESCUINO_STACK_NUM_MAX];
static uint8_t id_set[VESCUINO_STACK_NUM_MAX] = {0,};
static uint8_t comm_set[VESCUINO_STACK_NUM_MAX] = {0,};
static float value_set[VESCUINO_STACK_NUM_MAX] = {0.,};

//
#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
static THD_FUNCTION(spi_slave_read_thread, arg);
static THD_WORKING_AREA(spi_slave_read_thread_wa, 4096);
#endif
static THD_FUNCTION(custom_cmd_check_thread, arg);
static THD_WORKING_AREA(custom_cmd_check_thread_wa, 128);
static THD_FUNCTION(custom_cmd_debug_thread, arg);
static THD_WORKING_AREA(custom_cmd_debug_thread_wa, 512);


#endif /* APPLICATIONS_APP_VESCUINO_SPI_H_ */
