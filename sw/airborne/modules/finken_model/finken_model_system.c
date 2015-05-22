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
#include "subsystems/navigation/common_flight_plan.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_environment.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

// TODO: sane values
#ifndef FINKEN_SYSTEM_P
#define FINKEN_SYSTEM_P 0.075
#endif

#ifndef FINKEN_SYSTEM_I
#define FINKEN_SYSTEM_I 0.00
#endif

#ifndef FINKEN_THRUST_P
#define FINKEN_THRUST_P 0.1f
#endif

#ifndef FINKEN_THRUST_I
#define FINKEN_THRUST_I 0.0f
#endif

#ifndef FINKEN_THRUST_D
#define FINKEN_THRUST_D 0.00f
#endif


#ifndef FINKEN_BASE_THRUST
#define FINKEN_BASE_THRUST 0.5f
#endif

#ifndef FINKEN_SYSTEM_UPDATE_FREQ
#define FINKEN_SYSTEM_UPDATE_FREQ 30
#endif

#define SIZE 20

struct system_model_s finken_system_model;
struct system_model_s finken_system_set_point;

static float error_z[SIZE];

void update_actuators_set_point(void);

void finken_system_model_init(void) {
  finken_system_model.distance_z     = 0.0;
  finken_system_model.velocity_theta = 0.0;
  finken_system_model.velocity_x     = 0.0;
  finken_system_model.velocity_y     = 0.0;

	finken_actuators_set_point.alpha  = 0.0;
	finken_actuators_set_point.beta   = 0.0;
	finken_actuators_set_point.theta  = 0.0;
	finken_actuators_set_point.thrust = 0.0;

	for(unsigned int i=0;i<sizeof(error_z)/sizeof(float);i++)
		error_z[i]=0.0f;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_MODEL", send_finken_system_model_telemetry);
}

void finken_system_model_periodic(void)
{
	update_finken_system_model();
	update_actuators_set_point();
}

void update_finken_system_model(void)
{
	if(finken_sensor_model.distance_z < 2.5) {
		finken_system_model.distance_z     = finken_sensor_model.distance_z;
	}
	
  finken_system_model.velocity_theta = finken_sensor_model.velocity_theta;
  finken_system_model.velocity_x     = finken_sensor_model.velocity_x;
  finken_system_model.velocity_y     = finken_sensor_model.velocity_y;
}

/*
 * Use finken_system_set_point to calculate new actuator settings
 */


void update_actuators_set_point()
{
	/*
	// front , back
	float error_x =   finken_sensor_model.distance_d_front - finken_sensor_model.distance_d_back;
	// left , right
	float error_y =   finken_sensor_model.distance_d_left - finken_sensor_model.distance_d_right;

	finken_actuators_set_point.beta = error_x * FINKEN_SYSTEM_P;
	finken_actuators_set_point.alpha = error_y * FINKEN_SYSTEM_P;

	*/
	static int next=0;
	//if(autopilot_mode == AP_MODE_NAV && stage_time > 0) 
	//	error_z[next] = 0.0f; 
	//else 
		error_z[next] = finken_system_set_point.distance_z - finken_system_model.distance_z; 

	float sum_error_z = 0.0f;
	//float d_error_z = error_z[next] - error_z[(next-1)%SIZE];

	//for(unsigned int i=0;i<SIZE;i++)
	//	sum_error_z += error_z[i];

	//sum_error_z /= SIZE * FINKEN_SYSTEM_UPDATE_FREQ;

	finken_actuators_set_point.thrust =  error_z[next] * FINKEN_THRUST_P;
	//finken_actuators_set_point.thrust += sum_error_z   * FINKEN_THRUST_I;
	//finken_actuators_set_point.thrust += d_error_z     * FINKEN_THRUST_D;
	finken_actuators_set_point.thrust += FINKEN_BASE_THRUST;


	//next=(next+1)%SIZE;
}

void send_finken_system_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_SYSTEM_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_system_model.distance_z,
    &finken_system_model.velocity_theta,
    &finken_system_model.velocity_x,
    &finken_system_model.velocity_y,
    &finken_actuators_set_point.alpha,
    &finken_actuators_set_point.beta,
    &finken_actuators_set_point.thrust
  );
}

