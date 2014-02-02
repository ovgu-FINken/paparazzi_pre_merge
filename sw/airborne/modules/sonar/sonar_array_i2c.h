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

/** @file sonar_array_i2c.h
 *  @brief driver for the 5 sonar sensors of the ovgu-fink
 */

#ifndef SONAR_ARRAY_I2C_H
#define SONAR_ARRAY_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

struct sonar_values_s {
	uint16_t front;
	uint16_t right;
	uint16_t back;
	uint16_t left;
	uint16_t down;
};
extern struct sonar_values_s sonar_values;

struct sonar_data_available_s {
	bool_t front;
	bool_t right;
	bool_t back;
	bool_t left;
	bool_t down;
};

void sonar_send_command(uint8_t i2c_addr);
extern struct sonar_data_aviable_s sonar_data_aviable;

extern void sonar_array_i2c_init(void);
extern void sonar_array_i2c_periodic(void);
extern void sonar_array_i2c_event(void);


#endif
