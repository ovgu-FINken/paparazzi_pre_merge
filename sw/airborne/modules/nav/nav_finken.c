/*
 * Copyright (C) 2014 Sebastian Mai, ovgu
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

/**
 * @file modules/nav/nav_finken.h
 * @brief finken nav
 */

#include "subsystems/navigation/common_flight_plan.h"
#include "generated/flight_plan.h"

// Imu is required
#include "subsystems/imu.h"

#include "messages.h"
#include "subsystems/datalink/datalink.h"

#include "modules/sonar/sonar_array_i2c.h"

#ifndef SONAR_FAILSAFE_LIMIT
#define SONAR_FAILSAFE_LIMIT 200
#endif

#ifndef SONAR_FAILSAFE_RANGE
#define SONAR_FAILSAFE_RANGE 150
#endif

#ifndef SONAR_FAILSAFE_P
#define SONAR_FAILSAFE_P 0.3
#endif

#ifndef SONAR_FAILSAFE_D
#define SONAR_FAILSAFE_D 0.10
#endif

#ifndef SONAR_FAILSAFE_I
#define SONAR_FAILSAFE_I 0.0
#endif

double e_front_sum = 0;
double e_front_old = 0;
double e_back_sum  = 0;
double e_back_old  = 0;

double e_left_sum  = 0;
double e_left_old  = 0;

double e_right_sum = 0;
double e_right_old = 0;

double e_down_sum  = 0;
double e_down_old  = 0;

float sonar_failsave_pitch(void) {
	// New PID control

	// Sonar angle correctino
	//doule distanz_correct=0;
	//distanz_correct=cos(pitchangle)*sonar_values.front
	double t_a = 0.166;

	// Distanz front
	double e_front = SONAR_FAILSAFE_RANGE - sonar_values.front;
	e_front_sum = e_front_sum + e_front;
	e_front_old = SONAR_FAILSAFE_RANGE - sonar_values_old.front;

	float out_front = e_front * SONAR_FAILSAFE_P +
		((e_front - e_front_old) * SONAR_FAILSAFE_D / t_a) +
		SONAR_FAILSAFE_I * t_a * e_front_sum;

	if(sonar_values.front > SONAR_FAILSAFE_LIMIT)
	{
		out_front   = 0;
		e_front_sum = 0;
	}

	// Distanz back
	double e_back = SONAR_FAILSAFE_RANGE - sonar_values.back;
	e_back_sum = e_back_sum + e_back;
	e_back_old = SONAR_FAILSAFE_RANGE - sonar_values_old.back;

	float out_back = e_back * SONAR_FAILSAFE_P +
		((e_back - e_back_old) * SONAR_FAILSAFE_D / t_a) +
		SONAR_FAILSAFE_I * t_a * e_back_sum;

	if(sonar_values.back > SONAR_FAILSAFE_LIMIT)
	{
		out_back   = 0;
		e_back_sum = 0;
	}

	// Differenz between back and front
	double pitchangle = 0;
	if(sonar_values.front <= SONAR_FAILSAFE_LIMIT || sonar_values.back <= SONAR_FAILSAFE_LIMIT)
	{
		pitchangle = out_front - out_back;
	}

	return pitchangle;
}


float sonar_failsave_roll(void) {
	double t_a = 0.166;

	// Distanz left
	double e_left = SONAR_FAILSAFE_RANGE - sonar_values.left;
	e_left_sum = e_left_sum + e_left;

	float out_left = e_left * SONAR_FAILSAFE_P +
		((e_left - e_left_old) * SONAR_FAILSAFE_D / t_a) +
		SONAR_FAILSAFE_I * t_a * e_left_sum;

	e_left_old = e_left;

	if(sonar_values.left > SONAR_FAILSAFE_LIMIT)
	{
		out_left   = 0;
		e_left_sum = 0;
		e_left_old = e_left;
	}

	// Distanz right
	double e_right = SONAR_FAILSAFE_RANGE - sonar_values.right;
	e_right_sum = e_right_sum + e_right;

	float out_right = e_right * SONAR_FAILSAFE_P +
		((e_right - e_right_old) * SONAR_FAILSAFE_D / t_a) +
		SONAR_FAILSAFE_I * t_a * e_right_sum;

	e_right_old = e_right;
	if(sonar_values.right > SONAR_FAILSAFE_LIMIT)
	{
		out_right   = 0;
		e_right_sum = 0;
		e_right_old = e_right;
	}

	// Differenz between back and front
	double roll = 0;
	if(sonar_values.left <= SONAR_FAILSAFE_LIMIT || sonar_values.right <= SONAR_FAILSAFE_LIMIT)
	{
		roll = out_left - out_right;
	}

	return roll;
}
