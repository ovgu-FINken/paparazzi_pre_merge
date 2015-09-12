/*
 * Copyright (C) 2014 Andreas Pfohl
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/finken_model/finken_model_system.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_actuators.h"

#include "subsystems/radio_control.h"
#include "arch/stm32/subsystems/radio_control/spektrum_arch.h"

#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

#include "subsystems/datalink/downlink.h"

#ifndef FINKEN_VELOCITY_X_P
#define FINKEN_VELOCITY_X_P 0.05
#endif

#ifndef FINKEN_VELOCITY_Y_P
#define FINKEN_VELOCITY_Y_P 0.05
#endif

#ifndef FINKEN_VELOCITY_X_D
#define FINKEN_VELOCITY_X_D 0
#endif

#ifndef FINKEN_VELOCITY_Y_D
#define FINKEN_VELOCITY_Y_D 0
#endif

#ifndef FINKEN_VELOCITY_CONTROL_MODE
#define FINKEN_VELOCITY_CONTROL_MODE 0
#endif

#ifndef FINKEN_VELOCITY_DESIRED_X
#define FINKEN_VELOCITY_DESIRED_X 0	// m/sec
#endif

#ifndef FINKEN_VELOCITY_DESIRED_Y
#define FINKEN_VELOCITY_DESIRED_Y 0	// m/sec
#endif

struct system_model_s finken_system_set_point;
bool finken_system_model_control_height;

float thrust_k_dec1 = 0.0;
float thrust_k_dec2 = 0.0;
float error_z_k_dec1 = 0.0;
float error_z_k_dec2 = 0.0;

float error_x_p = 0.0;
float error_y_p = 0.0;
float error_x_d = 0.0;
float error_y_d = 0.0;

float temp_pitch = 0;
float temp_roll = 0;

void finken_system_model_init(void) {
  finken_system_set_point.z          = 0.0;
  finken_system_set_point.yaw        = 0.0;
	finken_system_model_control_height = 0;
  finken_system_set_point.velocity_x = FINKEN_VELOCITY_DESIRED_Y;
  finken_system_set_point.velocity_y = FINKEN_VELOCITY_DESIRED_X;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_SET_POINT", send_finken_system_set_point_telemetry);

}

/*
 * Use finken_system_set_point to calculate new actuator settings
 */

float takeoff_roll, takeoff_pitch;

void finken_system_model_periodic(void)
{	
	takeoff_roll = (float) radio_control.values[RADIO_ROLL] / 13000 * 10;
	takeoff_pitch= (float) radio_control.values[RADIO_PITCH] / 13000 * 10;

	float error_z_k =POS_FLOAT_OF_BFP(finken_system_set_point.z - finken_sensor_model.pos.z);

	float thrust_k = 1.6552f * thrust_k_dec1 - 0.6552f * thrust_k_dec2 + 209.0553f * error_z_k - 413.7859f * error_z_k_dec1 + 204.7450f * error_z_k_dec2;
	
	if( !finken_system_model_control_height )
		thrust_k = 0;

	error_z_k_dec2 = error_z_k_dec1;
	error_z_k_dec1 = error_z_k;
	
	thrust_k_dec2=thrust_k_dec1;
	thrust_k_dec1=thrust_k;

	if (FINKEN_THRUST_DEFAULT + thrust_k /100 < 0.2 || FINKEN_THRUST_DEFAULT + thrust_k / 100 > 0.8)
		thrust_k -= 2*(209.0553f - 413.7859f + 204.7450)*(error_z_k+error_z_k_dec1+error_z_k_dec2)/3;

	if(FINKEN_THRUST_DEFAULT + thrust_k / 100 < 0.2){
		finken_actuators_set_point.thrust = 0.2;
	}
	else if(FINKEN_THRUST_DEFAULT + thrust_k / 100 > 0.8){
		finken_actuators_set_point.thrust = 0.8;
		//TODO anti-windup
	}
	else{
		finken_actuators_set_point.thrust = FINKEN_THRUST_DEFAULT + thrust_k / 100;
	}
	error_x_p = (finken_system_set_point.velocity_x - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x)) * FINKEN_VELOCITY_X_P;
	error_y_p = (finken_system_set_point.velocity_y - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y)) * FINKEN_VELOCITY_Y_P;
	error_x_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x)) * FINKEN_VELOCITY_X_D;	//constant velocity
	error_y_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y)) * FINKEN_VELOCITY_Y_D;	//constant velocity

	temp_pitch = error_x_p + error_x_d;
	temp_roll = error_y_p + error_y_d;

	if(FINKEN_VELOCITY_CONTROL_MODE)	{
		finken_actuators_set_point.pitch = error_x_p + error_x_d;
		finken_actuators_set_point.roll = error_y_p + error_y_d;
	}
	else	{
	finken_actuators_set_point.pitch = 0.0;
	finken_actuators_set_point.roll  = 0.0;
	}

	// TODO: Theta
}

void send_finken_system_set_point_telemetry(struct transport_tx *trans, struct link_device* link)
{
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_SYSTEM_SET_POINT(
    DefaultChannel,
    DefaultDevice,
		&finken_system_set_point.z,
		&finken_system_set_point.yaw,
		&temp_pitch,	//finken_system_set_point.velocity_x,
		&temp_roll	//finken_system_set_point.velocity_y
  );
}
