/*
 * Copyright (C) 2014  Sebastian Mai
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

/** @file sonar_px4.h
 *  @brief simple driver to deal with one sonar sensor on px4
 */

#ifndef SONAR_PX4_H
#define SONAR_PX4_H

#include "std.h"
#include "modules/optical_flow/px4flow.h"

/** New data available.
 */
#define sonar_data_available optical_flow_aviable

/** Sonar distance in m.
 */
#define sonar_distance optical_flow.ground_distance

#define SonarEvent(_handler) { \
  if (sonar_data_available) { \
    _handler(); \
    sonar_data_available = FALSE; \
  } \
}

#endif
