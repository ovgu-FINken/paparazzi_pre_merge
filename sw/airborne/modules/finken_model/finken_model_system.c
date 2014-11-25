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
#include "subsystems/datalink/telemetry.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_environment.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

// TODO: sane values
#ifndef FINKEN_SYSTEM_P
#define FINKEN_SYSTEM_P 0.1
#endif

#ifndef FINKEN_SYSTEM_I
#define FINKEN_SYSTEM_I 0.00
#endif

#ifndef FINKEN_THRUST_P
#define FINKEN_THRUST_P /* 0.15 */0.15
#endif

#ifndef FINKEN_THRUST_I
#define FINKEN_THRUST_I /*  0.05  */0.05
#endif

#ifndef FINKEN_THRUST_DEFAULT
#define FINKEN_THRUST_DEFAULT /* 0.48 */0.48
#endif

#ifndef FINKEN_SYSTEM_UPDATE_FREQ
#define FINKEN_SYSTEM_UPDATE_FREQ 30
#endif

#ifndef FINKEN_VERTICAL_VELOCITY_FACTOR
#define FINKEN_VERTICAL_VELOCITY_FACTOR 0.02
#endif

struct system_model_s finken_system_model;
struct actuators_model_s finken_actuators_set_point;

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
float sum_error_x = 0;
float sum_error_y = 0;
float sum_error_z = 0;
float distance_z_old = 0.0; 


void update_actuators_set_point()
{
	float error_x = finken_system_model.velocity_x - finken_system_set_point.velocity_x;
	sum_error_x += error_x;

	finken_actuators_set_point.beta = (error_x * FINKEN_SYSTEM_P + sum_error_x * FINKEN_SYSTEM_I) * FINKEN_SYSTEM_UPDATE_FREQ;

	float error_y = finken_system_model.velocity_y - finken_system_set_point.velocity_y;
	sum_error_y += error_y;

	finken_actuators_set_point.alpha = (error_y * FINKEN_SYSTEM_P + sum_error_y * FINKEN_SYSTEM_I) * FINKEN_SYSTEM_UPDATE_FREQ;

	float error_z = finken_system_set_point.distance_z - finken_system_model.distance_z; 
	if(autopilot_mode == AP_MODE_NAV) 
	{
		sum_error_z += error_z;
	} 
	else 
	{
		sum_error_z = 0;
	}

	float velocity_z = (finken_system_model.distance_z - distance_z_old) * FINKEN_SYSTEM_UPDATE_FREQ;

	finken_actuators_set_point.thrust = FINKEN_THRUST_DEFAULT + error_z * FINKEN_THRUST_P;
	finken_actuators_set_point.thrust += sum_error_z * FINKEN_THRUST_I / FINKEN_SYSTEM_UPDATE_FREQ;

	finken_actuators_set_point.thrust -= FINKEN_VERTICAL_VELOCITY_FACTOR * (velocity_z / (sqrt(1 + velocity_z * velocity_z)));

	if(finken_actuators_set_point.thrust < 0.2)
		finken_actuators_set_point.thrust = 0.2;
	else if(finken_actuators_set_point.thrust > 1.0)
		finken_actuators_set_point.thrust = 1.0;

	distance_z_old = finken_system_model.distance_z;

	// TODO: Theta
}

void send_finken_system_model_telemetry(void)
{
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

