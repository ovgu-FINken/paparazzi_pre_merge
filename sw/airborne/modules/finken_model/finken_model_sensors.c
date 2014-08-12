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

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"

/* sensors */
#include "modules/sonar/sonar_array_i2c.h"
#include "modules/optical_flow/px4flow.h"

struct sensor_model_s finken_sensor_model;

void finken_sensor_model_init()
{
  finken_sensor_model.distance_z       = &optical_flow.ground_distance;
  finken_sensor_model.distance_d_front = &sonar_values.front;
  finken_sensor_model.distance_d_right = &sonar_values.right;
  finken_sensor_model.distance_d_back  = &sonar_values.back;
  finken_sensor_model.distance_d_left  = &sonar_values.left;
//  finken_sensor_model.acceleration_x   = NULL;
//  finken_sensor_model.acceleration_y   = NULL;
//  finken_sensor_model.acceleration_z   = NULL;
//  finken_sensor_model.velocity_alpha   = NULL;
//  finken_sensor_model.velocity_beta    = NULL;
//  finken_sensor_model.velocity_theta   = NULL;
  finken_sensor_model.velocity_x       = &optical_flow.flow_comp_m_x;
  finken_sensor_model.velocity_y       = &optical_flow.flow_comp_m_y;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SENSOR_MODEL", send_finken_sensor_model_telemetry);
}

void send_finken_sensor_model_telemetry() {
  DOWNLINK_SEND_FINKEN_SENSOR_MODEL(
    DefaultChannel,
    DefaultDevice,
    finken_sensor_model.distance_z,
    finken_sensor_model.distance_d_front,
    finken_sensor_model.distance_d_right,
    finken_sensor_model.distance_d_back,
    finken_sensor_model.distance_d_left,
//    finken_sensor_model.acceleration_x,
//    finken_sensor_model.acceleration_y,
//    finken_sensor_model.acceleration_z,
//    finken_sensor_model.velocity_alpha,
//    finken_sensor_model.velocity_beta,
//    finken_sensor_model.velocity_theta,
    finken_sensor_model.velocity_x,
    finken_sensor_model.velocity_y
  );
}
