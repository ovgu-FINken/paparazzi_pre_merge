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
#include "state.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_simple_matrix.h"


// TODO: sane values
#ifndef FINKEN_SYSTEM_P
#define FINKEN_SYSTEM_P 0.075
#endif

#ifndef FINKEN_SYSTEM_I
#define FINKEN_SYSTEM_I 0.00
#endif

#ifndef FINKEN_THRUST_P
#define FINKEN_THRUST_P /* 0.15 */0.10
#endif

#ifndef FINKEN_THRUST_I
#define FINKEN_THRUST_I /*  0.05  */0.05
#endif

#ifndef FINKEN_VELOCITY_X_P
#define FINKEN_VELOCITY_X_P 0.05
#endif

#ifndef FINKEN_VELOCITY_Y_P
#define FINKEN_VELOCITY_Y_P 0.05
#endif

#ifndef FINKEN_VELOCITY_X_D
#define FINKEN_VELOCITY_X_D 0.5
#endif

#ifndef FINKEN_VELOCITY_Y_D
#define FINKEN_VELOCITY_Y_D 0.5
#endif


#ifndef FINKEN_SYSTEM_UPDATE_FREQ
#define FINKEN_SYSTEM_UPDATE_FREQ 30
#endif

#ifndef FINKEN_SYSTEM_DT
#define FINKEN_SYSTEM_DT 1/FINKEN_SYSTEM_UPDATE_FREQ
#endif

#ifndef FINKEN_VERTICAL_VELOCITY_FACTOR
#define FINKEN_VERTICAL_VELOCITY_FACTOR 0.04
#endif

#ifndef FINKEN_VELOCITY_CONTROL_MODE
#define FINKEN_VELOCITY_CONTROL_MODE 0
#endif

#ifndef FINKEN_VELOCITY_X
#define FINKEN_VELOCITY_X 0.5	// m/sec
#endif

#ifndef FINKEN_VELOCITY_Y
#define FINKEN_VELOCITY_Y 0.5	// m/sec --> ACCEL_BFP_OF_REAL
#endif

struct system_model_s finken_system_model;
struct actuators_model_s finken_actuators_set_point;

void update_actuators_set_point(void);

void finken_system_model_init(void) {
  finken_system_model.distance_z     = 0.0;
  finken_system_model.velocity_theta = 0.0;
  finken_system_model.velocity_x     = 0.0;
  finken_system_model.velocity_y     = 0.0;

	finken_system_model.acceleration_x = 0.0;
	finken_system_model.acceleration_y = 0.0;
	finken_system_model.acceleration_z = 0.0;

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

//float local_accel[3][1];	
//float global_accel[3][1];
struct Int32Eulers *angles; //roll = phi; pitch = theta; jaw = psi
struct Int32RMat R;
struct Int32Eulers *angels_t;
struct Int32Vect3 global_accel;
struct Int32Vect3 local_accel;
struct NedCoor_i *ned_accel_i;
//struct Int32Eulers angles;

void update_finken_system_model(void)
{
	if(finken_sensor_model.distance_z < 2.5) {
		finken_system_model.distance_z     = finken_sensor_model.distance_z;
	}

	//if(!bit_is_set(state.accel_status, ACCEL_NED_I))	{
		ned_accel_i = stateGetAccelNed_i();
		local_accel.x   = ned_accel_i->x;	//north
		local_accel.y   = ned_accel_i->y;	//east
		local_accel.z   = 0;//ned_accel_i->z;	//down	
	//}
	/*else{
 		local_accel.x  = 0.0;
 		local_accel.y   = 0.0;
 		local_accel.z   = 0.0;
	}*/

	angles = stateGetNedToBodyEulers_i();
	int32_rmat_of_eulers_321(&R, angles);	//from local to global

	//int32_rmat_transp_vmult(&global_accel, &R, &local_accel);	//transform local acceleration into global coordinate system
	global_accel.x = local_accel.x;
	global_accel.y = local_accel.y;


	finken_system_model.velocity_x    += global_accel.x * FINKEN_SYSTEM_DT;//(((global_accel.x) >> 10) + ((global_accel.x) % 1024)) * FINKEN_SYSTEM_DT;//finken_sensor_model.acceleration_x * FINKEN_SYSTEM_DT;
	finken_system_model.velocity_y    += global_accel.y * FINKEN_SYSTEM_DT;//(((global_accel.y) >> 10) + ((global_accel.y) % 1024)) * FINKEN_SYSTEM_DT;//finken_sensor_model.acceleration_y * FINKEN_SYSTEM_DT;

	finken_system_model.velocity_theta = finken_sensor_model.velocity_theta;

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

	if(FINKEN_VELOCITY_CONTROL_MODE)	{

		float error_x_p = (FINKEN_VELOCITY_X - finken_system_model.velocity_x) * FINKEN_VELOCITY_X_P;
		float error_y_p = (FINKEN_VELOCITY_Y - finken_system_model.velocity_y) * FINKEN_VELOCITY_Y_P;
		float error_x_d = (0 - finken_system_model.acceleration_x) * FINKEN_VELOCITY_X_D;	//constant velocity
		float error_y_d = (0 - finken_system_model.acceleration_y) * FINKEN_VELOCITY_Y_D;	//constant velocity

		finken_actuators_set_point.beta = error_x_p + error_x_d;
		finken_actuators_set_point.alpha = error_y_p + error_y_d;
	}
	else	{
		/* front , back */
		float error_x =   finken_sensor_model.distance_d_front - finken_sensor_model.distance_d_back;
		/* left , right */
		float error_y =   finken_sensor_model.distance_d_left - finken_sensor_model.distance_d_right;

		finken_actuators_set_point.beta = error_x * FINKEN_SYSTEM_P;
		finken_actuators_set_point.alpha = error_y * FINKEN_SYSTEM_P;
	}


	float error_z = finken_system_set_point.distance_z - finken_system_model.distance_z; 
	if(autopilot_mode == AP_MODE_NAV && stage_time > 0) 
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


	distance_z_old = finken_system_model.distance_z;

	// TODO: Theta
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

