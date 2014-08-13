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

/** @file finken_model_system.h
 *  @brief module for the system
 */

#ifndef FINKEN_MODEL_SYSTEM_H
#define FINKEN_MODEL_SYSTEM_H

#include "std.h"
#include "modules/finken_model/finken_model_actuators.h"

struct system_model_s {
  float distance_z;
  float velocity_theta;
  float velocity_x;
  float velocity_y;
};

extern struct system_model_s finken_system_model;
extern struct actuators_model_s finken_actuators_set_point;

extern void finken_system_model_init();
extern void finken_system_model_periodic();
void update_finken_system_model();

extern void send_finken_system_model_telemetry();

#endif