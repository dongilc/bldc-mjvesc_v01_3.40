/*
 * app_vescuino_interface.c
 * (modified from bldc_interface.c)
 *
 *  Created on: 19 Feb, 2018
 *      Author: cdi
 */

#ifndef APPLICATIONS_APP_VESCUINO_INTERFACE_H_
#define APPLICATIONS_APP_VESCUINO_INTERFACE_H_

#include "buffer.h"
#include <string.h>
#include "app_vescuino.h"

// Private variables
static unsigned char send_buffer[512];
static int32_t can_fwd_vesc = -1;
static volatile uint8_t spi_id = 0;

// Private variables for received data
static mc_values values;
static int fw_major;
static int fw_minor;
static float rotor_pos;
static float detect_cycle_int_limit;
static float detect_coupling_k;
static signed char detect_hall_table[8];
static signed char detect_hall_res;
static float dec_ppm;
static float dec_ppm_len;
static float dec_adc;
static float dec_adc_voltage;
static float dec_chuk;

//cdi
typedef struct {
	uint8_t ch;
	int fw_major;
	int fw_minor;
    float erpm;
    int tachometer;
    int tachometer_abs;
} custom_values;
static custom_values c_values;
can_status_msg *can_st_msg;

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len);
static void fwd_can_append(uint8_t *data, int32_t *ind);

// Function pointers
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_printf_func)(char *str) = 0;
static void(*rx_fw_func)(int major, int minor) = 0;
static void(*rx_rotor_pos_func)(float pos) = 0;
static void(*rx_mcconf_func)(mc_configuration *conf) = 0;
static void(*rx_appconf_func)(app_configuration *conf) = 0;
static void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res) = 0;
static void(*rx_dec_ppm_func)(float val, float ms) = 0;
static void(*rx_dec_adc_func)(float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(float val) = 0;
static void(*rx_mcconf_received_func)(void) = 0;
static void(*rx_appconf_received_func)(void) = 0;
static void(*rx_custom_app_data_func)(custom_values *c_values) = 0;	//cdi

#endif /* APPLICATIONS_APP_VESCUINO_INTERFACE_H_ */
