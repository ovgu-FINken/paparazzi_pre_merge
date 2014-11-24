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

#include "modules/finken_model/finken_model_environment.h"
#include "subsystems/datalink/telemetry.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include <math.h>

struct environment_model_s finken_environment_model;
struct system_model_s finken_system_set_point;

void finken_environment_model_init() {
  finken_environment_model.alpha    = 0.0;
  finken_environment_model.distance = 0;
	finken_system_set_point.distance_z     = 0.0;
	finken_system_set_point.velocity_theta = 0.0;
	finken_system_set_point.velocity_x     = 0.0;
	finken_system_set_point.velocity_y     = 0.0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_ENVIRONMENT_MODEL", send_finken_environment_model_telemetry);
}

void finken_environment_model_periodic(){
  int16_t distance = finken_sensor_model.distance_d_front;
  uint16_t alpha_offset = 0;
	if(finken_sensor_model.distance_d_left < finken_sensor_model.distance_d_right) {
		alpha_offset = 270;
	}

  if(finken_sensor_model.distance_d_right < distance)
  {
    distance = finken_sensor_model.distance_d_right;
    alpha_offset = 90;
		if(finken_sensor_model.distance_d_front < finken_sensor_model.distance_d_back) {
			alpha_offset = 0;
		}
  }

  if(finken_sensor_model.distance_d_back < distance)
  {
    distance = finken_sensor_model.distance_d_back;
    alpha_offset = 180;
		if(finken_sensor_model.distance_d_right < finken_sensor_model.distance_d_left) {
			alpha_offset = 90;
		}
  }

  if(finken_sensor_model.distance_d_left < distance)
  {
    distance = finken_sensor_model.distance_d_left;
    alpha_offset = 270;
		if(finken_sensor_model.distance_d_back < finken_sensor_model.distance_d_front) {
			alpha_offset = 180;
		}
  }

	
	float alpha;
	switch(alpha_offset) {
		case 0:
			alpha = atan((float)(finken_sensor_model.distance_d_left) / finken_sensor_model.distance_d_front);
			finken_environment_model.distance = sin(alpha) * finken_sensor_model.distance_d_left;
			finken_environment_model.alpha = alpha_offset + alpha;
			break;
		case 90:
			alpha = atan((float)(finken_sensor_model.distance_d_front) / finken_sensor_model.distance_d_right);
			finken_environment_model.distance = sin(alpha) * finken_sensor_model.distance_d_front;
			finken_environment_model.alpha = alpha_offset + alpha;
			break;
		case 180:
			alpha = atan((float)(finken_sensor_model.distance_d_right) / finken_sensor_model.distance_d_back);
			finken_environment_model.distance = sin(alpha) * finken_sensor_model.distance_d_right;
			finken_environment_model.alpha = alpha_offset + alpha;
			break;
		case 270:
			alpha = atan((float)(finken_sensor_model.distance_d_back) / finken_sensor_model.distance_d_left);
			finken_environment_model.distance = sin(alpha) * finken_sensor_model.distance_d_back;
			finken_environment_model.alpha = alpha_offset + alpha;
			break;
	}
}

void send_finken_environment_model_telemetry()
{
  DOWNLINK_SEND_FINKEN_ENVIRONMENT_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_environment_model.alpha,
    &finken_environment_model.distance
  );
}
