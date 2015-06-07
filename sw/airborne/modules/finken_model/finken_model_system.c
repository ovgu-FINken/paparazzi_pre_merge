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

#include "modules/finken_model/finken_model_pid.h"


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
struct actuators_model_s finken_actuators_set_point;

void update_actuators_set_point(void);

//all init
float DEG_TO_GRAD_COEFF = 0.01745329251; //math.pi / 180;
float MAX_PROXY_DIST = 5.00;		//max measurable distance from the sonar
float MAX_IR_DIST = 2.00;			//max measurable distance from IR sensor
float TOLERABLE_PROXY_DIST = 0.7;	//minimum treshold - if less, begin collision avoidance
float FLIGHT_HEIGHT = 1.30;			//the target altitude we will try to maintain at all times
float MIN_HEIGHT = 0.75;			//the minimum altitude before we can begin using sonars

//float proxy_dist =

//end init

void init_pid()
{
	--[[storage for measured distances--]]
	proxyDist = {}	-- current readings from FRONT, BACK, RIGHT and LEFT sensor (in that order)
	oldProxyDist = {}
	--indexing in lua begins from 1 by default, but arrays are actually index-value hashtables and even a negative index is allowed
	for i=1,4 do
		proxyDist[i] = _MAX_PROXY_DIST
		oldProxyDist[i] = proxyDist[i]
	end
	irDist = 0	-- current readings from the IR sensor
	oldIRDist = irDist
	-- end-of-sensor-storage

	--[[Quadbot--]]
	local object1 = simGetObjectHandle('SimFinken')
	script = simGetScriptAssociatedWithObject(object1)

	--[[Controllers--]]
	-- z-axis control: convert IR readings to throttle in order to maintain stable height
	zPIDController = newPIDController(1, 0, 1) -- zero integral coeff, because limiting the PID output will mess up the integral part
	zPIDController.setMinMax(-30, 30)	-- upper and lower bounds, avoid bursts
	-- x,y-axis controllers:
	_CMode_NONE = 0;
	_CModeLinear = 1;	-- SIMULATION ONLY! For testing the zPIDController, of no importance for the final task
	_CModeDirectPID = 2;
	controllerMode = _CModeDirectPID
		--[[ SIMULATION ONLY! A dummy-controller to create simple x,y-flight so we can focus only on testing how the zPIDController works
			1) if distance from wall is too small (<= _TOLERABLE_PROXY_DIST): tilt robot against it, begin retreat
			2) if distance from wall is moderate: level the robot to horizontal position
			3) if distance from wall is too big (>= 2.5*_TOLERABLE_PROXY_DIST): tilt robot toward it, begin approach
		--]]
		xLinearController = newLinearController(script, 'pitch', _TOLERABLE_PROXY_DIST, 2.5*_TOLERABLE_PROXY_DIST, -5, 10)
		--yLinearController = newLinearController(script, 'roll', _TOLERABLE_PROXY_DIST, 2.5*_TOLERABLE_PROXY_DIST, -5, 5)
		--end-of-simulation-dummy-controller
	--[[ PID Controller: try to control roll and pitch directly from the measured distance --]]
	xPID_Direct = newDirectDistToAngleController({4.7,0,6.9}, _TOLERABLE_PROXY_DIST)
	xPID_Direct.setMinMax(-6, 6)
	--yPID_Direct = newDirectDistToAngleController({4.7,0,6.9}, _TOLERABLE_PROXY_DIST)
	--yPID_Direct.setMinMax(-8, 8)
	-- end-of-controllers

	 --[[Handles for sensors--]]
	local proxyHandleNames = {
		'SimFinken_sensor_front',
		'SimFinken_sensor_back',
		'SimFinken_sensor_right',
		'SimFinken_sensor_left'
	}
	proxyHandles = {}
	for i=1,4 do
		proxyHandles[i] = simGetObjectHandle(proxyHandleNames[i])
	end
	irHandle = simGetObjectHandle('SimFinken_sensor_IR')
	--end-of-handles
}

void finken_system_model_init(void)
{
   finken_system_model.distance_z     = 0.0;
   finken_system_model.velocity_theta = 0.0;
   finken_system_model.velocity_x     = 0.0;
   finken_system_model.velocity_y     = 0.0;

   finken_actuators_set_point.alpha  = 0.0;
   finken_actuators_set_point.beta   = 0.0;
   finken_actuators_set_point.theta  = 0.0;
   finken_actuators_set_point.thrust = 0.0;

   //Todo init pid

   init_pid();

   register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_MODEL", send_finken_system_model_telemetry);
}

void finken_system_model_periodic(void)
{
	update_finken_system_model();
	update_actuators_set_point();
	//Todo update pid
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
	/* front , back */
	float error_x =   finken_sensor_model.distance_d_front - finken_sensor_model.distance_d_back;
	/* left , right */
	float error_y =   finken_sensor_model.distance_d_left - finken_sensor_model.distance_d_right;

	finken_actuators_set_point.beta = error_x * FINKEN_SYSTEM_P;
	finken_actuators_set_point.alpha = error_y * FINKEN_SYSTEM_P;


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

//require "utilitiesFun"
//
//if (sim_call_type==sim_childscriptcall_initialization) then
//
//	--[[global constants (anything that is not marked as 'local' is globally accessible--]]
//	_DEG_TO_GRAD_COEFF = math.pi / 180
//	_MAX_PROXY_DIST = 5.00	-- max measurable distance from the sonar
//	_MAX_IR_DIST = 2.00		-- max measurable distance from IR sensor
//	_TOLERABLE_PROXY_DIST = 0.7	-- minimum treshhold - if less, begin collision avoidance
//	_FLIGHT_HEIGHT = 1.30	-- the target altitude we will try to maintain at all times
//	_MIN_HEIGHT = 0.75		-- the minimum altitude before we can begin using sonars
//
//	-- SIMULATION ONLY: lower resolution from 50ms (20Hz) to 250ms (4Hz) to simulate worse sensor update rate
//	-- fix for the problem of v-rep not working correctly if we change the timestep from the drop-down box
//	_UPDATE_RATE_COEFF = 5	-- WARNING: change "local _UPDATE_RATE_COEFF = 5" in "utilitiesFun.lua" if you change the value here!
//	tickCounter = 0
//	-- end-of-update-rate-control-variables
//
//	--[[storage for measured distances--]]
//	proxyDist = {}	-- current readings from FRONT, BACK, RIGHT and LEFT sensor (in that order)
//	oldProxyDist = {}
//	--indexing in lua begins from 1 by default, but arrays are actually index-value hashtables and even a negative index is allowed
//	for i=1,4 do
//		proxyDist[i] = _MAX_PROXY_DIST
//		oldProxyDist[i] = proxyDist[i]
//	end
//	irDist = 0	-- current readings from the IR sensor
//	oldIRDist = irDist
//	-- end-of-sensor-storage
//
//	--[[Quadbot--]]
//	local object1 = simGetObjectHandle('SimFinken')
//	script = simGetScriptAssociatedWithObject(object1)
//
//	--[[Controllers--]]
//	-- z-axis control: convert IR readings to throttle in order to maintain stable height
//	zPIDController = newPIDController(1, 0, 1) -- zero integral coeff, because limiting the PID output will mess up the integral part
//	zPIDController.setMinMax(-30, 30)	-- upper and lower bounds, avoid bursts
//	-- x,y-axis controllers:
//	_CMode_NONE = 0;
//	_CModeLinear = 1;	-- SIMULATION ONLY! For testing the zPIDController, of no importance for the final task
//	_CModeDirectPID = 2;
//	controllerMode = _CModeDirectPID
//		--[[ SIMULATION ONLY! A dummy-controller to create simple x,y-flight so we can focus only on testing how the zPIDController works
//			1) if distance from wall is too small (<= _TOLERABLE_PROXY_DIST): tilt robot against it, begin retreat
//			2) if distance from wall is moderate: level the robot to horizontal position
//			3) if distance from wall is too big (>= 2.5*_TOLERABLE_PROXY_DIST): tilt robot toward it, begin approach
//		--]]
//		xLinearController = newLinearController(script, 'pitch', _TOLERABLE_PROXY_DIST, 2.5*_TOLERABLE_PROXY_DIST, -5, 10)
//		--yLinearController = newLinearController(script, 'roll', _TOLERABLE_PROXY_DIST, 2.5*_TOLERABLE_PROXY_DIST, -5, 5)
//		--end-of-simulation-dummy-controller
//	--[[ PID Controller: try to control roll and pitch directly from the measured distance --]]
//	xPID_Direct = newDirectDistToAngleController({4.7,0,6.9}, _TOLERABLE_PROXY_DIST)
//	xPID_Direct.setMinMax(-6, 6)
//	--yPID_Direct = newDirectDistToAngleController({4.7,0,6.9}, _TOLERABLE_PROXY_DIST)
//	--yPID_Direct.setMinMax(-8, 8)
//	-- end-of-controllers
//
//	 --[[Handles for sensors--]]
//	local proxyHandleNames = {
//		'SimFinken_sensor_front',
//		'SimFinken_sensor_back',
//		'SimFinken_sensor_right',
//		'SimFinken_sensor_left'
//	}
//	proxyHandles = {}
//	for i=1,4 do
//		proxyHandles[i] = simGetObjectHandle(proxyHandleNames[i])
//	end
//	irHandle = simGetObjectHandle('SimFinken_sensor_IR')
//	--end-of-handles
//end
//
//
//if (sim_call_type==sim_childscriptcall_actuation) then
//
//	-- SIMULATION ONLY: Use to simulate lower update rate
//	tickCounter = tickCounter + 1
//	if(tickCounter > _UPDATE_RATE_COEFF) then
//		tickCounter = 1
//	end
//	if(tickCounter > 1) then
//		return
//	end
//	--end-of-update-rate-control
//
//	local target, curr, error
//	local dt = simGetSimulationTimeStep() -- timestep between calls - system update frequency
//	dt = dt * _UPDATE_RATE_COEFF	-- SIMULATION ONLY: update-rate control
//
//	--[[height control--]]
//	target = (_FLIGHT_HEIGHT - irDist)/dt
//	curr = (irDist - oldIRDist)/dt
//	error = target - curr
//	local targetThrottle = zPIDController.adjust(error)
//	simSetScriptSimulationParameter(script,'throttle', 50 + targetThrottle)
//	-- end-of-height-control
//
//	--[[TURN OFF x-y control until safe altitude is reached--]]
//	if(irDist > _MIN_HEIGHT) then
//		if(controllerMode == _CModeLinear) then
//			xLinearController.adjust(proxyDist[1])
//		elseif(controllerMode == _CModeDirectPID) then
//			local pitch = xPID_Direct.adjust(proxyDist[1])
//			simSetScriptSimulationParameter(script, 'pitch', pitch)
//		end
//	end
//	-- end-of-x-control
//end
//
//--[[WARNING: simulation is simplified, IGNORE problems regarding real-time behaviour
//	1) assume all sensors readings are precise (no need for filtering)
//	2) any computation is immediate (zero delay)
//--]]
//if (sim_call_type==sim_childscriptcall_sensing) then
//
//	-- SIMULATION ONLY: Use to simulate lower update rate
//	if(tickCounter > 1) then
//		return
//	end
//	-- end-of-update-rate-control
//
//	local pitch = simGetScriptSimulationParameter(script,'pitch') * _DEG_TO_GRAD_COEFF
//	local roll = simGetScriptSimulationParameter(script,'roll') * _DEG_TO_GRAD_COEFF
//	local cosX = math.cos(pitch)
//	local cosY = math.cos(roll)
//
//	local res,dist,coeff
//	-- sonar readings
//	for i=1,4 do
//		res,dist = simReadProximitySensor(proxyHandles[i])
//		if(res == 1) then
//			coeff = cosX
//			if(i >= 3) then
//				coeff = cosY
//			end
//			oldProxyDist[i] = proxyDist[i]
//			proxyDist[i] = coeff*dist
//		end
//	end
//
//	-- IR height readings
//	res,dist = simReadProximitySensor(irHandle)
//	if(res == 1) then
//		local tgX = math.tan(roll)
//		local tgY = math.tan(pitch)
//		coeff = 1 / math.sqrt(1 + tgX^2 + tgY^2)
//		oldIRDist = irDist
//		irDist = dist*coeff
//	end
//end
//
//
//if (sim_call_type==sim_childscriptcall_cleanup) then
//end
