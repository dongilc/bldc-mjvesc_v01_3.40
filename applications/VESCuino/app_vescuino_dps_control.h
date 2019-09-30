/*
 * app_vescuino_dps_control.h
 *
 *  Created on: Feb 7, 2019
 *      Author: cdi
 */

#ifndef APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_
#define APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_

//
void app_vescuino_dps_set_enc_offset(float offset);
float app_vescuino_dps_target(void);
float app_vescuino_dps_actual(void);
float app_vescuino_dps_s_prof(void);
float app_vescuino_dps_s_actual(void);
double app_vescuino_dps_tacho_prof(void);
double app_vescuino_dps_tacho_actual(void);
double app_vescuino_dps_deg_target(void);

#endif /* APPLICATIONS_VESCUINO_APP_VESCUINO_DPS_CONTROL_H_ */
