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

#include "subsystems/radio_control.h"
#include "arch/stm32/subsystems/radio_control/spektrum_arch.h"

#include "finken_model_pid.h"
#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

#include "subsystems/datalink/downlink.h"

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

#ifndef FINKEN_SYSTEM_UPDATE_FREQ
#define FINKEN_SYSTEM_UPDATE_FREQ 30
#endif

#ifndef FINKEN_VERTICAL_VELOCITY_FACTOR
#define FINKEN_VERTICAL_VELOCITY_FACTOR 0.04
#endif

struct system_model_s finken_system_model;
struct system_model_s finken_system_set_point;

void update_actuators_set_point(void);

//all init
float DEG_TO_GRAD_COEFF = 0.01745329251; //math.pi / 180;
float MAX_PROXY_DIST = 5.00;		//max measurable distance from the sonar
float MAX_IR_DIST = 2.00;			//max measurable distance from IR sensor
float TOLERABLE_PROXY_DIST = 80;			//minimum treshold - if less, begin collision avoidance
float FLIGHT_HEIGHT = 1.30;	//the target altitude we will try to maintain at all times
float MIN_HEIGHT = 0.15;	//the minimum altitude before we can begin using sonars

struct pid_controller zPIDController;

struct pid_controller frontPIDController;
struct pid_controller rightPIDController;
struct pid_controller backPIDController;
struct pid_controller leftPIDController;

//this is for creating the different pids and assigning minmax-values to them.
void init_pid() {
	initWallController(&frontPIDController);
	initWallController(&rightPIDController);
	initWallController(&backPIDController);
	initWallController(&leftPIDController);
}

void finken_system_model_init(void) {
	finken_system_model.distance_z = 0.0;
	finken_system_model.velocity_theta = 0.0;
	finken_system_model.velocity_x = 0.0;
	finken_system_model.velocity_y = 0.0;

	finken_actuators_set_point.alpha = 0.0;
	finken_actuators_set_point.beta = 0.0;
	finken_actuators_set_point.theta = 0.0;
	finken_actuators_set_point.thrust = 0.0;

	init_pid();

	register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_MODEL", send_finken_system_model_telemetry);
	register_periodic_telemetry(DefaultPeriodic, "X_PID", send_x_pid_telemetry);
	register_periodic_telemetry(DefaultPeriodic, "FLOAT_DEBUG", send_float_pid_telemetry);
}

void finken_system_model_periodic(void) {
	update_finken_system_model();
	update_actuators_set_point();
}

void update_finken_system_model(void) {
	if (finken_sensor_model.distance_z < 2.5) {
		finken_system_model.distance_z = finken_sensor_model.distance_z;
	}

	finken_system_model.velocity_theta = finken_sensor_model.velocity_theta;
	finken_system_model.velocity_x = finken_sensor_model.velocity_x;
	finken_system_model.velocity_y = finken_sensor_model.velocity_y;
}

/*
 * Use finken_system_set_point to calculate new actuator settings
 */
float sum_error_x = 0;
float sum_error_y = 0;
float sum_error_z = 0;
float distance_z_old = 0.0;

float oldIRDist = 0;

//This method is for the height controller. Since we will use a different one, it is useless, but not yet deleted. :D
float pid_thrust(float irDist) {
	float target, curr, error;

	float dt = 0.1;	//Todo find correct time step

	// height control
	target = (FLIGHT_HEIGHT - irDist) / dt;
	curr = (irDist - oldIRDist) / dt;
	error = target - curr;
	float targetThrottle = adjust(error, 1 / FINKEN_SYSTEM_UPDATE_FREQ, &zPIDController);
	return 50 + targetThrottle;
}

//pid_controller for x and y
//currently, it will only controll the first sonar-parameter (front or left), which is just for testing. should include an integration
//of front-back and left-right
float max_(float x, float y) {
	return (x < y) ? y : x;
}

float pid_planar(float sonar_dist, struct pid_controller *pid) {
	float error = max_(TOLERABLE_PROXY_DIST - sonar_dist, 0.0);
	return adjust(error, 0.03, pid);		//return pitch or roll
}

void update_actuators_set_point() {
	finken_actuators_set_point.beta = (float) radio_control.values[RADIO_ROLL] / 13000 * 10;
	finken_actuators_set_point.alpha = (float) radio_control.values[RADIO_PITCH] / 13000 * 10;

	float error_z = finken_system_set_point.distance_z - finken_system_model.distance_z;
	if (autopilot_mode == AP_MODE_NAV && stage_time > 0) {
		sum_error_z += error_z;
	} else {
		sum_error_z = 0;
	}

	float velocity_z = (finken_system_model.distance_z - distance_z_old) * FINKEN_SYSTEM_UPDATE_FREQ;

	finken_actuators_set_point.thrust = FINKEN_THRUST_DEFAULT + error_z * FINKEN_THRUST_P;
	//finken_actuators_set_point.thrust += sum_error_z * FINKEN_THRUST_I / FINKEN_SYSTEM_UPDATE_FREQ;
	//finken_actuators_set_point.thrust -= FINKEN_VERTICAL_VELOCITY_FACTOR * (velocity_z / (sqrt(1 + velocity_z * velocity_z)));


//	if (finken_system_model.distance_z > MIN_HEIGHT) {
//		float front = pid_planar(finken_sensor_model.distance_d_front, &frontPIDController);
//		float back = pid_planar(finken_sensor_model.distance_d_back, &backPIDController);
//		float xDegree = ((front - back) / 141) * 12;
//		finken_actuators_set_point.alpha += xDegree;	//pitch
//
//		float left = pid_planar(finken_sensor_model.distance_d_left, &leftPIDController);
//		float right = pid_planar(finken_sensor_model.distance_d_right, &rightPIDController);
//		float yDegree = ((left - right) / 141) * 12;
//		finken_actuators_set_point.beta += yDegree;	//roll
//	}

	distance_z_old = finken_system_model.distance_z;

	// TODO: Theta
}

void send_finken_system_model_telemetry(struct transport_tx *trans, struct link_device* link) {
	trans = trans;
	link = link;
	DOWNLINK_SEND_FINKEN_SYSTEM_MODEL(DefaultChannel, DefaultDevice, &finken_system_model.distance_z, &finken_system_model.velocity_theta, &finken_system_model.velocity_x,
			&finken_system_model.velocity_y, &finken_actuators_set_point.alpha, &finken_actuators_set_point.beta, &finken_actuators_set_point.thrust,
			&finken_system_set_point.distance_z);
}

void send_x_pid_telemetry(struct transport_tx *trans, struct link_device *link) {
	trans = trans;
	link = link;
	DOWNLINK_SEND_X_PID(DefaultChannel, DefaultDevice, &frontPIDController.t, &frontPIDController.pPart, &frontPIDController.iPart, &frontPIDController.dPart,
			&frontPIDController.previousError, &frontPIDController.res);
}
void send_float_pid_telemetry(struct transport_tx *trans, struct link_device *link) {
	trans = trans;
	link = link;
	DOWNLINK_SEND_FLOAT_DEBUG(DefaultChannel, DefaultDevice, &yFinkenFloatController.t, &yFinkenFloatController.pPart, &yFinkenFloatController.iPart, &yFinkenFloatController.dPart,
			&yFinkenFloatController.previousError, &yFinkenFloatController.res);
}

