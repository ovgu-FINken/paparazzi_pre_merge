/*
 * Copyright (C) 2014 Sebastian Mai
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

#include "modules/finken_model/finken_model_actuators.h"

struct actuators_model_s finken_actuators_model;

void finken_actuators_model_init() {
	finken_actuators_model.alpha = 0;
	finken_actuators_model.beta  = 0;
	finken_actuators_model.theta = 0;
	finken_actuators_model.thrust= 0;
}

