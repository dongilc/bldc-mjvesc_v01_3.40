/*
 * app_diff_robot_control.c
 *
 *  Created on: 2018. 8. 27.
 *      Author: cdi
 */

//
static CONT_VAR_MG cont;
static nunchuk_data_norm chuk_d_norm;

//
static THD_FUNCTION(diff_robot_control_thread, arg);
static THD_WORKING_AREA(diff_robot_control_thread_wa, 1024);

/*
 * Differential Robot Control Program
 */
// cdi
//
void app_diff_robot_thread_start(void)
{
	//
	chThdCreateStatic(diff_robot_control_thread_wa, sizeof(diff_robot_control_thread_wa), NORMALPRIO, diff_robot_control_thread, NULL);
	debug_printf("\r\nDifferential Robot Control Thread Activated");
}

// SMG Control Thread
static THD_FUNCTION(diff_robot_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("diff_robot_cont");
	chThdSleepMilliseconds(2000);

	static bool en_flag = 0;
	static uint32_t bt_c_click_cnt = 0;
	static bool bt_c_hold_flag = 0;

	for(;;) {
		// ############## Button Control ##############
		app_nunchuk_get_normalized_data(&chuk_d_norm);

		// 'button C click' is Enable off
		// 'button C hold' is Enable on
		if (chuk_d_norm.bt_c==0) {
			bt_c_hold_flag = 0;
			bt_c_click_cnt = 0;
		} else if (chuk_d_norm.bt_c==1) {
			if (bt_c_hold_flag == 0) bt_c_click_cnt++;

			// button C click
			if (bt_c_click_cnt == 5 && en_flag==0) {
				// Control On
				en_flag = 1;
				commands_printf("Diff Robot Control On");
			}

			// button C hold, 1sec
			if(bt_c_click_cnt >= 100) {
				bt_c_hold_flag = 1;

				en_flag = 0;

				memset(&cont, 0.0, sizeof(CONT_VAR_MG));
				mc_interface_set_current(0.);
				comm_can_set_current(1, 0.);

				commands_printf("Diff Robot Control Off");

				bt_c_click_cnt = 0;
			}
		}

		// 'button Z click'
		if (chuk_d_norm.bt_z==1) {
			commands_printf("BT-Z Clicked");
		}
		// ############## End of Button Control ##############

		// On Control State
		if(en_flag == 1)
		{
			// Out
			cont.duty_1 = 0.4*(chuk_d_norm.js_x);
			app_util_cdi_lowpass_filter(cont.duty_1, &cont.duty_1_lpf, 0.3, 0.01);
			cont.duty_2 = -0.7*(chuk_d_norm.js_y);
			app_util_cdi_lowpass_filter(cont.duty_2, &cont.duty_2_lpf, 0.5, 0.01);

			// Control Out
			mc_interface_set_duty(cont.duty_1_lpf);
			comm_can_set_duty(1, cont.duty_2_lpf);
		}
		else
		{
			mc_interface_set_current(0.);
			comm_can_set_current(1, 0.);
		}

		// Loop Time Manage
		chThdSleepMilliseconds(10);
	}

}
