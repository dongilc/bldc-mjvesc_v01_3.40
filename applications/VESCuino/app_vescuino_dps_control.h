/*
 * app_vescuino_dps_control.h
 *
 *  Created on: Feb 7, 2019
 *      Author: cdi
 */

#ifndef APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_
#define APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_

//
int app_vescuino_dps_get_dps_set_flag(void);
void app_vescuino_dps_set_dps_set_flag(int);
void app_vescuino_dps_set_enc_offset(float offset);
float app_vescuino_dps_get_enc_offset(void);
//
void app_vescuino_dps_set_vel_maximum(float max);
void app_vescuino_dps_set_acc_maximum(float max);
float app_vescuino_dps_get_vel_maximum(void);
float app_vescuino_dps_get_acc_maximum(void);
//
void app_vescuino_dps_set_goto_kp(float kp);
float app_vescuino_dps_get_goto_kp(void);
//
float app_vescuino_dps_target(void);
float app_vescuino_dps_actual(void);
float app_vescuino_dps_s_prof(void);
float app_vescuino_dps_s_actual(void);
double app_vescuino_dps_tacho_prof(void);
double app_vescuino_dps_tacho_actual(void);
double app_vescuino_dps_deg_target(void);

void app_vescuino_set_dps_timeout_override_flag(int);

#endif /* APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_ */
