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
#include "subsystems/datalink/telemetry.h"

/* input */
#include "modules/sonar/sonar_array_i2c.h"
#include "modules/optical_flow/px4flow.h"
#include "state.h"

struct sensor_model_s finken_sensor_model;

void finken_sensor_model_init()
{
  finken_sensor_model.distance_z       = 0.0;
  finken_sensor_model.distance_d_front = 0;
  finken_sensor_model.distance_d_right = 0;
  finken_sensor_model.distance_d_back  = 0;
  finken_sensor_model.distance_d_left  = 0;
  finken_sensor_model.acceleration_x   = 0.0;
  finken_sensor_model.acceleration_y   = 0.0;
  finken_sensor_model.acceleration_z   = 0.0;
  finken_sensor_model.velocity_alpha   = 0.0;
  finken_sensor_model.velocity_beta    = 0.0;
  finken_sensor_model.velocity_theta   = 0.0;
  finken_sensor_model.velocity_x       = 0.0;
  finken_sensor_model.velocity_y       = 0.0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SENSOR_MODEL", send_finken_sensor_model_telemetry);
}

void finken_sensor_model_periodic()
{
  finken_sensor_model.distance_z       = optical_flow.ground_distance;
  finken_sensor_model.distance_d_front = sonar_values.front;
  finken_sensor_model.distance_d_right = sonar_values.right;
  finken_sensor_model.distance_d_back  = sonar_values.back;
  finken_sensor_model.distance_d_left  = sonar_values.left;

	struct NedCoor_f* accel = stateGetAccelNed_f();
  finken_sensor_model.acceleration_x   = accel->x;
  finken_sensor_model.acceleration_y   = accel->y;
  finken_sensor_model.acceleration_z   = accel->z;

  finken_sensor_model.velocity_alpha   = 0.0;
  finken_sensor_model.velocity_beta    = 0.0;
  finken_sensor_model.velocity_theta   = 0.0;

  finken_sensor_model.velocity_x       = optical_flow.flow_comp_m_x;
  finken_sensor_model.velocity_y       = optical_flow.flow_comp_m_y;
}

void send_finken_sensor_model_telemetry() {
  DOWNLINK_SEND_FINKEN_SENSOR_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_sensor_model.distance_z,
    &finken_sensor_model.distance_d_front,
    &finken_sensor_model.distance_d_right,
    &finken_sensor_model.distance_d_back,
    &finken_sensor_model.distance_d_left,
    &finken_sensor_model.acceleration_x,
    &finken_sensor_model.acceleration_y,
    &finken_sensor_model.acceleration_z,
    &finken_sensor_model.velocity_alpha,
    &finken_sensor_model.velocity_beta,
    &finken_sensor_model.velocity_theta,
    &finken_sensor_model.velocity_x,
    &finken_sensor_model.velocity_y
  );
}
