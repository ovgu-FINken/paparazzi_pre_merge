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
#include "modules/finken_ir_adc/finken_ir_adc.h"

struct sensor_model_s finken_sensor_model;

void finken_sensor_model_init(void)
{
  finken_sensor_model.distance_z       = 0.0;
  finken_sensor_model.distance_d_front = 0;
  finken_sensor_model.distance_d_right = 0;
  finken_sensor_model.distance_d_back  = 0;
  finken_sensor_model.distance_d_left  = 0;
  //finken_sensor_model.acceleration_x   = 0.0;
  //finken_sensor_model.acceleration_y   = 0.0;
  //finken_sensor_model.acceleration_z   = 0.0;
  finken_sensor_model.velocity_alpha   = 0.0;
  finken_sensor_model.velocity_beta    = 0.0;
  finken_sensor_model.velocity_theta   = 0.0;
  finken_sensor_model.velocity_x       = 0.0;
  finken_sensor_model.velocity_y       = 0.0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SENSOR_MODEL", send_finken_sensor_model_telemetry);
}

void finken_sensor_model_periodic(void)
{
  finken_sensor_model.distance_z       = ir_distance_equalized;
	if(sonar_values.front != 765)
		finken_sensor_model.distance_d_front = sonar_values.front;
	if(sonar_values.right != 765)
		finken_sensor_model.distance_d_right = sonar_values.right;
	if(sonar_values.back != 765)
		finken_sensor_model.distance_d_back  = sonar_values.back;
	if(sonar_values.left != 765)
		finken_sensor_model.distance_d_left  = sonar_values.left;
  //finken_sensor_model.acceleration_x   = 0.0;
  //finken_sensor_model.acceleration_y   = 0.0;
  //finken_sensor_model.acceleration_z   = 0.0;
  finken_sensor_model.velocity_alpha   = 0.0;
  finken_sensor_model.velocity_beta    = 0.0;
  finken_sensor_model.velocity_theta   = 0.0;
  finken_sensor_model.velocity_x       = 0.0;
  finken_sensor_model.velocity_y       = 0.0;
}

void send_finken_sensor_model_telemetry(struct transport_tx *trans, struct link_device* link) {
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_SENSOR_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_sensor_model.distance_z,
    &finken_sensor_model.distance_d_front,
    &finken_sensor_model.distance_d_right,
    &finken_sensor_model.distance_d_left,
    &finken_sensor_model.distance_d_back
    //&finken_sensor_model.acceleration_x,
    //&finken_sensor_model.acceleration_y,
    //&finken_sensor_model.acceleration_z
  );
}
