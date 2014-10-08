/*
 * Copyright (C) 2014  Sebastian Mai, Andreas Pfohl
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
 */

/** @file sonar_array_i2c.h
 *  @brief driver for the 5 sonar sensors of the ovgu-fink
 */

#ifndef SONAR_ARRAY_I2C_H
#define SONAR_ARRAY_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

struct sonar_values_s {
	int16_t front;
	int16_t right;
	int16_t back;
	int16_t left;
};


extern struct sonar_values_s sonar_values;
extern struct sonar_values_s sonar_values_old;

extern void sonar_array_i2c_init();
extern void sonar_array_i2c_periodic();
extern void sonar_array_i2c_event();
extern void send_sonar_array_telemetry();

void sonar_send_command(uint8_t i2c_addr);
void query_all_sensors();
void query_sensor(int16_t *value, int16_t *old_value, uint8_t i2c_addr, struct i2c_transaction *transaction);

#endif
