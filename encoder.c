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

#include "encoder.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "utils.h"
#include "math.h"
#include "mc_interface.h"	//cdi
#include "mcpwm_foc.h"		//cdi
#include "VESCuino/app_vescuino_dps_control.h"	//cdi

//cdi
#define	RPM2RPS		2.*M_PI/60.0
#define RAD2DEG		180.0/M_PI
#define DEG2RAD		M_PI/180.0
#define REV2RAD		2.*M_PI
//#define POLEPAIR	10		// 4inch Benz Motor
//#define ERPM2RPS	1./POLEPAIR*RPM2RPS
//#define TACHO2REV	1./(6.*POLEPAIR)

//cdi
// Custom Encoder use.
#ifdef USE_CUSTOM_ABI_ENCODER_AT_SPI
#define HW_ENC_A_PORT				HW_SPI_PORT_MISO	// HW_SPI_PORT_SCK @VER0.1A
#define HW_ENC_A_PIN				HW_SPI_PIN_MISO		// HW_SPI_PIN_SCK @VER0.1A
#define HW_ENC_B_PORT				HW_SPI_PORT_MOSI	// HW_SPI_PORT_MISO @VER0.1A
#define HW_ENC_B_PIN				HW_SPI_PIN_MOSI		// HW_SPI_PIN_MISO @VER0.1A
#define HW_ENC_I_PORT				HW_SPI_PORT_NSS		// HW_SPI_PORT_NSS @VER0.1A
#define HW_ENC_I_PIN				HW_SPI_PIN_NSS		// HW_SPI_PIN_NSS @VER0.1A
#else
#define HW_ENC_I_PORT				HW_HALL_ENC_GPIO3
#define HW_ENC_I_PIN				HW_HALL_ENC_PIN3
#endif

// Defines
#define AS5047P_READ_ANGLECOM		(0x3FFF | 0x4000 | 0x8000) // This is just ones
#define AS5047_SAMPLE_RATE_HZ		20000

#if AS5047_USE_HW_SPI_PINS
#ifdef HW_SPI_DEV
#define SPI_SW_MISO_GPIO			HW_SPI_PORT_MISO
#define SPI_SW_MISO_PIN				HW_SPI_PIN_MISO
#define SPI_SW_MOSI_GPIO			HW_SPI_PORT_MOSI
#define SPI_SW_MOSI_PIN				HW_SPI_PIN_MOSI
#define SPI_SW_SCK_GPIO				HW_SPI_PORT_SCK
#define SPI_SW_SCK_PIN				HW_SPI_PIN_SCK
#define SPI_SW_CS_GPIO				HW_SPI_PORT_NSS
#define SPI_SW_CS_PIN				HW_SPI_PIN_NSS
#else
// Note: These values are hardcoded.
#define SPI_SW_MISO_GPIO			GPIOB
#define SPI_SW_MISO_PIN				4
#define SPI_SW_MOSI_GPIO			GPIOB
#define SPI_SW_MOSI_PIN				5
#define SPI_SW_SCK_GPIO				GPIOB
#define SPI_SW_SCK_PIN				3
#define SPI_SW_CS_GPIO				GPIOB
#define SPI_SW_CS_PIN				0
#endif
#else
#define SPI_SW_MISO_GPIO			HW_HALL_ENC_GPIO2
#define SPI_SW_MISO_PIN				HW_HALL_ENC_PIN2
#define SPI_SW_SCK_GPIO				HW_HALL_ENC_GPIO1
#define SPI_SW_SCK_PIN				HW_HALL_ENC_PIN1
#define SPI_SW_CS_GPIO				HW_HALL_ENC_GPIO3
#define SPI_SW_CS_PIN				HW_HALL_ENC_PIN3
#endif

#ifdef USE_ENC_CALCULATION_THREAD
	static THD_FUNCTION(enc_thread, arg);
	static THD_WORKING_AREA(enc_thread_wa, 1024);
#endif

// Private types
typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI
} encoder_mode;

// Private variables
static bool index_found = false;
static bool hall_enc_hybrid_switch = false;
static uint32_t enc_counts = 10000;
static encoder_mode mode = ENCODER_MODE_NONE;
static float last_enc_angle = 0.0;

//cdi
static float enc_dt = 0.00005;	// 20kHz
static int	 end_dir = 1;
static float enc_deg_input = 0.0;
static float enc_deg_prev = 0.0;
static float enc_deg_now = 0.0;
static float enc_deg_diff = 0.0;
static float enc_dps = 0.0;
static float enc_dps_lpf = 0.0;
static int   motor_type = 0;
static bool  enc_inverted;
static float enc_ratio = 0.;
static float enc_erpm2rps = 0.;
static float enc_tacho2rev = 0.;
static float enc_rps = 0.0;
static float enc_rad = 0.0;
#ifdef USE_ENC_CALCULATION_THREAD
static float enc_pll_phase = 0.0;
static float enc_pll_speed = 0.0;
#endif

// Private functions
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_INPUT_PULLUP);

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);

	index_found = false;
	mode = ENCODER_MODE_NONE;
	last_enc_angle = 0.0;
}

void encoder_init_abi(uint32_t counts) {
	EXTI_InitTypeDef   EXTI_InitStructure;

	// Initialize variables
	//cdi
#ifdef USE_CUSTOM_ABI_ENCODER_AT_SPI
	index_found = true;	// cheat as if encoder index is already found
	hall_enc_hybrid_switch = false;
#else
	index_found = false;
#endif
	enc_counts = counts;

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI	//cdi
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
//	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
#else	//cdi
	palSetPadMode(HW_ENC_A_PORT, HW_ENC_A_PIN, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
	palSetPadMode(HW_ENC_B_PORT, HW_ENC_B_PIN, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
#endif

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	TIM_EncoderInterfaceConfig (HW_ENC_TIM, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising,
			TIM_ICPolarity_Rising);
	TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);

	// Filter
	HW_ENC_TIM->CCMR1 |= 6 << 12 | 6 << 4;
	HW_ENC_TIM->CCMR2 |= 6 << 4;

	TIM_Cmd(HW_ENC_TIM, ENABLE);

	// Interrupt on index pulse

	// Connect EXTI Line to pin
	SYSCFG_EXTILineConfig(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);

	// Configure EXTI Line
	EXTI_InitStructure.EXTI_Line = HW_ENC_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(HW_ENC_EXTI_CH, 0);

	mode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
#if AS5047_USE_HW_SPI_PINS
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);
#endif

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	mode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
}

bool encoder_is_configured(void) {
	return mode != ENCODER_MODE_NONE;
}

bool encoder_is_hall_enc_switched(void) {
	return hall_enc_hybrid_switch;
}

float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (mode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = last_enc_angle;
		break;

	default:
		break;
	}

#ifdef USE_CUSTOM_ABI_ENCODER_AT_SPI	//cdi
	angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
#endif

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	// Only reset if the pin is still high to avoid too short pulses, which
	// most likely are noise.
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	//if (palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)) {
	if (palReadPad(HW_ENC_I_PORT, HW_ENC_I_PIN)) {	//cdi
		const unsigned int cnt = HW_ENC_TIM->CNT;
		static int bad_pulses = 0;
		const unsigned int lim = enc_counts / 20;

		if (index_found) {
			// Some plausibility filtering.
			if (cnt > (enc_counts - lim) || cnt < lim) {
				HW_ENC_TIM->CNT = 0;
				bad_pulses = 0;
			} else {
				bad_pulses++;

				if (bad_pulses > 5) {
					index_found = 0;
				}
			}
		} else {
			HW_ENC_TIM->CNT = 0;
			index_found = true;
			bad_pulses = 0;
		}

#ifdef USE_CUSTOM_ABI_ENCODER_AT_SPI
		//cdi
		if ((index_found==true) && (hall_enc_hybrid_switch==false))
		{
			app_vescuino_dps_set_enc_offset((double)mc_interface_get_pid_pos_now());

			HW_ENC_TIM->CNT = 0;
			index_found = true;
			bad_pulses = 0;

			//change foc_sensor_mode from hall to encoder just after encoder index found
			mcpwm_foc_change_sensor_mode_encoder();
			hall_enc_hybrid_switch = true;
		}
#endif
	}
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void) {
	uint16_t pos;

	spi_begin();
	spi_transfer(&pos, 0, 1);
	spi_end();

	pos &= 0x3FFF;
	last_enc_angle = ((float)pos * 360.0) / 16384.0;

#ifndef USE_ENC_CALCULATION_THREAD
	//cdi
	enc_deg_input = last_enc_angle;			// encoder raw signal
	enc_deg_now = enc_deg_input - 180.0;	// encoder -pi<=enc<=pi
	enc_deg_diff = utils_angle_difference(enc_deg_now, enc_deg_prev);
	enc_dps = enc_deg_diff / enc_dt;	// 20kH
	enc_deg_prev = enc_deg_now;

	//
	end_dir = (enc_inverted ? -1.0 : 1.0);
	enc_rps = end_dir*enc_dps*DEG2RAD;
	enc_rad += enc_rps*enc_dt;
#endif
}

#ifdef USE_ENC_CALCULATION_THREAD
//cdi
static void enc_pll_run(float phase, float dt, float *phase_var, float *speed_var)
{
	// smaller value is less noiser.
	static float pll_Kp = 200.0;//2000.0;
	static float pll_Ki = 4000.0;//40000.0;

	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + pll_Kp * delta_theta) * dt;
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += pll_Ki * delta_theta * dt;
}

//cdi
static THD_FUNCTION(enc_thread, arg) {
	(void)arg;

	chRegSetThreadName("enc_thread");

	enc_dt = 0.001;	// 1kH
	enc_deg_prev = -180.;

	for (;;) {
		//cdi
		enc_deg_input = encoder_read_deg();		// encoder raw signal
		enc_deg_now = enc_deg_input - 180.0;	// encoder -pi<=enc<=pi
		enc_deg_diff = utils_angle_difference(enc_deg_now, enc_deg_prev);
		enc_dps = enc_deg_diff / enc_dt;
		enc_deg_prev = enc_deg_now;

		//
		end_dir = (enc_inverted ? -1.0 : 1.0);
		enc_rps = enc_dps*DEG2RAD;
		enc_rad += enc_rps*enc_dt;	// pure integral

		// pll(rad[0~360], dt, rad_out, rps_out)
		enc_pll_run(enc_deg_input*DEG2RAD, enc_dt, &enc_pll_phase, &enc_pll_speed);

		chThdSleepMicroseconds(1000);
	}
}
#endif

//cdi
void encoder_init_dps(int type, float ratio, bool inv)
{
	motor_type = type;
	enc_inverted = inv;
	enc_ratio = ratio;

	enc_erpm2rps = 1./enc_ratio*RPM2RPS;
	enc_tacho2rev = 1./(6.*enc_ratio);

#ifdef USE_ENC_CALCULATION_THREAD
	chThdCreateStatic(enc_thread_wa, sizeof(enc_thread_wa), NORMALPRIO, enc_thread, NULL);
#endif
}

float encoder_read_deg_diff(void) {
	return enc_deg_diff;
}

float encoder_read_dps(void) {
	return enc_dps;
}

float encoder_read_rps(void) {
	float rps_out = 0.;

#ifdef USE_ENC_CALCULATION_THREAD
	rps_out = enc_pll_speed;//enc_rps;
#else
	rps_out = mc_interface_get_rpm()*enc_erpm2rps;
#endif

	return rps_out;
}

float encoder_read_rad(void) {
	float rad_out = 0.;

#ifdef USE_ENC_CALCULATION_THREAD
	rad_out = enc_rad;
#else
	rad_out = mc_interface_get_tachometer_value(false)*enc_tacho2rev*REV2RAD;
#endif

	return rad_out;
}

void encoder_data_reset(void) {
	mc_interface_get_tachometer_value(true);

	enc_deg_input = 0.0;
	enc_deg_prev = 0.0;
	enc_deg_now = 0.0;
	enc_deg_diff = 0.0;
	enc_dps = 0.0;
	enc_dps_lpf = 0.0;
	enc_rps = 0.0;
	enc_rad = 0.0;
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

// Software SPI
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay();
			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			int r1, r2, r3;
			r1 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r2 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r3 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
