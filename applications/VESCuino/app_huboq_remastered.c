/*
 * app_huboq_remastered.c
 *
 *  Created on: Nov 1, 2019
 *      Author: cdi
 */

//
#include "app_huboq_remastered.h"
#include "mc_interface.h"

static int cnt = 0;
static int find_home_cnt = 0;
static int find_home_flag = 0;	// 0:none, 1:finding home, 2:find success, 3:fail to find
static int finding_direction = 0;
volatile int limit_sw_detected_flag = 0;

//
static THD_FUNCTION(huboq_control_thread, arg);
static THD_WORKING_AREA(huboq_control_thread_wa, 1024);

//
void app_huboq_remastered_thread_start(void)
{
	chThdCreateStatic(huboq_control_thread_wa, sizeof(huboq_control_thread_wa), NORMALPRIO, huboq_control_thread, NULL);
	debug_printf("\r\nSET_HuboQ Remastered Thread Activated");
}

void app_huboq_set_limit_sw_status(int flag) {
    limit_sw_detected_flag = flag;
	
	if(find_home_flag==1) {
		if(encoder_is_configured() && encoder_index_found() && encoder_is_hall_enc_switched()) 
		{
			find_home_flag = 2;	// find success
		}
	}
}

int app_huboq_get_limit_sw_status(void) {
    return limit_sw_detected_flag;
}

void app_huboq_limit_sw_init(void) {
    // PB10 - limit sw
	EXTI_InitTypeDef   EXTI_InitStructure;

    // Enable SYSCFG clock    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // GPIO pin mode setting
    palSetPadMode(GPIOB, 10, PAL_MODE_INPUT_PULLUP);

	// Connect EXTI Line to pin
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);

	// Configure EXTI Line
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(EXTI15_10_IRQn, 0);

    find_home_flag = 0;
}

void app_huboq_find_home(void) {
	if(palReadPad(GPIOB, 10)) finding_direction = -1;
	else finding_direction = 1;

    find_home_flag = 1;
	limit_sw_detected_flag = 0;
    cnt = 0;
}

// huboq Thread
static THD_FUNCTION(huboq_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("huboq_control_thread");
	chThdSleepMilliseconds(1000);
	
    // time variables
	// static systime_t time_start;
	// static systime_t time_prev;
	// static systime_t time_duration;
	// uint32_t duration;

	for(;;) {
		// // Timer implementation
		// time_prev = time_start;
		// time_start = chVTGetSystemTime();
		// time_duration = time_start - time_prev;
		// duration = ST2US(time_duration);	// usec

        if(find_home_flag==1) {
            //if(!limit_sw_detected_flag) 
			{
                if(cnt<=300)      { mc_interface_set_duty(finding_direction*0.03); timeout_reset(); }
                else if(cnt<=600) { mc_interface_set_duty(finding_direction*-0.03); timeout_reset(); }
                else  {
					find_home_cnt++;
					cnt = 0;
				}
            }
            cnt++;
        }
		else if(find_home_flag==2) {
			if(cnt>0) debug_printf("\r\nSuccessed to find home");
			app_vescuino_set_dps(0);	// Position Lock
			timeout_reset();
			find_home_cnt = 0;
			cnt = 0;
		}
		else if(find_home_flag==3) {
			debug_printf("\r\nFailed to find home");
			find_home_flag = 0;
			find_home_cnt = 0;
			cnt = 0;
		}     

		// find trial over
		if(find_home_cnt>=2) {
			/* code */
			find_home_flag = 3;	
			find_home_cnt = 0;
			cnt = 0;
		}  
        
		chThdSleepMilliseconds(10);	// 100hz
	}
}