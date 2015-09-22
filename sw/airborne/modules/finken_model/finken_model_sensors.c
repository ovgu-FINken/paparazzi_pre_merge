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

#include "modules/finken_model/finken_model_sensors.h"

/* input */
#include "modules/sonar/sonar_array_i2c.h"
#include "state.h"
#include "subsystems/imu.h"
//#include "modules/finken_ir_adc/finken_ir_adc.h"
#include "modules/optical_flow/px4flow.h"

struct sensor_model_s finken_sensor_model;
struct Int32Eulers finken_sensor_attitude;
int64_t temp_mult;
uint32_t last_ts;

float pos_z;
/*enum {pos_z_size = 1};
static float pos_z_data[pos_z_size];
static uint8_t pos_z_i=0;*/
static const float maxZ = 3.0f;

void finken_sensor_model_init(void)
{
  memset(&finken_sensor_model, 0, sizeof(struct sensor_model_s));
	//memset(&pos_z_data, 0, sizeof(float)*pos_z_size);
	pos_z=0.0f;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SENSOR_MODEL", send_finken_sensor_model_telemetry);
  register_periodic_telemetry(DefaultPeriodic, "HC_DEBUG", send_finken_hc_debug);
}

void finken_sensor_model_periodic(void)
{
  // current timestamp
  uint32_t now_ms = get_sys_time_msec();

  memcpy(&finken_sensor_model.rates, &imu.gyro, sizeof(struct Int32Rates));
	/* x = y and y = x because of the coord. transformation from sensor to body coord. system */
	finken_sensor_model.acceleration.x = imu.accel.y;
	finken_sensor_model.acceleration.y = -imu.accel.x;
	finken_sensor_model.acceleration.z = imu.accel.z;
//  memcpy(&finken_sensor_model.acceleration, &imu.accel, sizeof(struct Int32Vect3));
	//finken_sensor_model.acceleration.x = sensor_filtered.acceleration.x;

	/*finken_sensor_model.distance_d_front = sonar_values.front;
	finken_sensor_model.distance_d_right = sonar_values.right;
	finken_sensor_model.distance_d_back  = sonar_values.back;
	finken_sensor_model.distance_d_left  = sonar_values.left;*/

	if(optical_flow.ground_distance < maxZ){
		pos_z/*_data[pos_z_i++]*/ = optical_flow.ground_distance;
		//pos_z_i%=pos_z_size;
	}

	/*for(uint8_t i = 0;i<pos_z_size;i++)
		pos_z+=pos_z_data[i];
  pos_z /= pos_z_size;*/

  finken_sensor_model.pos.z            = POS_BFP_OF_REAL(pos_z);
	memcpy(&finken_sensor_model.attitude, stateGetNedToBodyQuat_i(), sizeof(struct Int32Quat));
	int32_eulers_of_quat(&finken_sensor_attitude, &finken_sensor_model.attitude);

	/* x = -y and y = x because of the coord. transformation from sensor to body coord. system */	
	//finken_sensor_model.velocity.x       = SPEED_BFP_OF_REAL(optical_flow.flow_comp_m_x);
	finken_sensor_model.velocity.x       = SPEED_BFP_OF_REAL(-optical_flow.flow_comp_m_y);
	//finken_sensor_model.velocity.y       = SPEED_BFP_OF_REAL(optical_flow.flow_comp_m_y);
	finken_sensor_model.velocity.y       = SPEED_BFP_OF_REAL(optical_flow.flow_comp_m_x);

	if (now_ms > last_ts) {
		//fraction for position: 8, for speed 19 --> difference is 11
		temp_mult = (((int64_t)finken_sensor_model.velocity.x / (1<<11)) * (now_ms - last_ts));
		finken_sensor_model.pos.x += ((temp_mult / 1000));

		temp_mult = (((int64_t)finken_sensor_model.velocity.y) * (now_ms - last_ts)) / (1<<11);
		finken_sensor_model.pos.y += ((temp_mult / 1000));

		last_ts = now_ms;
	}
}

void send_finken_hc_debug(struct transport_tx *trans, struct link_device* link) {
  trans=trans;
  link=link;
  DOWNLINK_SEND_HC_DEBUG(
    DefaultChannel,
    DefaultDevice,
		&pos_z,
		&finken_actuators_set_point.thrust,
		&finken_actuators_set_point.pitch,
		&finken_actuators_set_point.roll,
		&finken_actuators_set_point.yaw
	);
}

void send_finken_sensor_model_telemetry(struct transport_tx *trans, struct link_device* link) {
  trans=trans;
  link=link;

	float pos_x      = POS_FLOAT_OF_BFP(finken_sensor_model.pos.x);
	float pos_y      = POS_FLOAT_OF_BFP(finken_sensor_model.pos.y);
	//float pos_z      = POS_FLOAT_OF_BFP(finken_sensor_model.pos.z);
	float pos_pitch  = ANGLE_FLOAT_OF_BFP(attitude.theta);
	float pos_roll   = ANGLE_FLOAT_OF_BFP(attitude.phi);
	float pos_yaw    = ANGLE_FLOAT_OF_BFP(attitude.psi);
	float velocity_x = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x);
	float velocity_y = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y);
	float velocity_z = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.z);
	float rate_pitch = RATE_FLOAT_OF_BFP(finken_sensor_model.rates.p);
	float rate_roll  = RATE_FLOAT_OF_BFP(finken_sensor_model.rates.q);
	float rate_yaw   = RATE_FLOAT_OF_BFP(finken_sensor_model.rates.r);
	float accel_x    = ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x);
	float accel_y    = ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y);
	float accel_z    = ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.z);

  DOWNLINK_SEND_FINKEN_SENSOR_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_sensor_model.distance_d_front,
    &finken_sensor_model.distance_d_right,
    &finken_sensor_model.distance_d_left,
    &finken_sensor_model.distance_d_back,
    &pos_x,      &pos_y,      &pos_z,
    &pos_pitch,  &pos_roll,   &pos_yaw,
    &velocity_x, &velocity_y, &velocity_z,
		&rate_pitch, &rate_roll,  &rate_yaw,
    &accel_x,    &accel_y,    &accel_z
  );
}
