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

#include "subsystems/nav.h"
#include "generated/flight_plan.h"

// Imu is required
#include "subsystems/imu.h"

#include "messages.h"
#include "subsystems/datalink/datalink.h"

#include "modules/sonar/sonar_array_i2c.h"

#ifndef SONAR_FAILSAVE_RANGE
#define SONAR_FAILSAVE_RANGE 150
#endif

#ifndef SONAR_FAILSAVE_P
#define SONAR_FAILSAVE_P 0.4
#endif

#ifndef SONAR_FAILSAVE_D
#define SONAR_FAILSAVE_D 0.0
#endif

#ifndef SONAR_FAILSAVE_I
#define SONAR_FAILSAVE_I 0.0
#endif

double e_front_sum = 0;
double e_front_old = 0;
double e_back_sum  = 0;
double e_back_old  = 0;

double e_left_sum  = 0;
double e_left_old  = 0;

double e_right_sum = 0;
double e_right_old = 0;

double e_down_sum = 0;
double e_down_old = 0;

float sonar_failsave_pitch() {
	// New PID control

	// Sonar angle correction
	//double distance_correct = 0;
	//distance_correct = cos(pitchangle) * sonar_values_filtered.front

	double t_a = 0.166;

	// Distance front
	double e_front = SONAR_FAILSAVE_RANGE - sonar_values_filtered.front;
	e_front_sum    = e_front_sum + e_front;
	e_front_old    = SONAR_FAILSAVE_RANGE - sonar_values_filtered_old.front;

	float out_front = e_front * SONAR_FAILSAVE_P +
		((e_front - e_front_old) * SONAR_FAILSAVE_D / t_a) +
		SONAR_FAILSAVE_I * t_a * e_front_sum;

	if(sonar_values_filtered.front > 120)
	{
		out_front   = 0;
		e_front_sum = 0;
	}

	// Distance back
	double e_back = SONAR_FAILSAVE_RANGE - sonar_values_filtered.back;
	e_back_sum    = e_back_sum + e_back;
	e_back_old    = SONAR_FAILSAVE_RANGE - sonar_values_filtered_old.back;

	float out_back = e_back * SONAR_FAILSAVE_P +
		((e_back - e_back_old) * SONAR_FAILSAVE_D / t_a) +
		SONAR_FAILSAVE_I * t_a * e_back_sum;

	if(sonar_values_filtered.back > 120)
	{
		out_back   = 0;
		e_back_sum = 0;
	}

	// Difference between back and front
	double pitchangle = 0;
	if(sonar_values_filtered.front <= 120 || sonar_values_filtered.back <= 120)
	{
		pitchangle = out_front - out_back;
	}

	return pitchangle;
}

float sonar_failsave_roll() {
	double t_a = 0.05;

	// Distanz left
	double e_left = SONAR_FAILSAVE_RANGE - sonar_values_filtered.left;
	e_left_sum    = e_left_sum + e_left;

	float out_left = e_left * SONAR_FAILSAVE_P +
		((e_left - e_left_old) * SONAR_FAILSAVE_D / t_a) +
		SONAR_FAILSAVE_I * t_a * e_left_sum;

	e_left_old = e_left;

	if(sonar_values_filtered.left > 120)
	{
		out_left   = 0;
		e_left_sum = 0;
		e_left_old = e_left;
	}

	// Distance right
	double e_right = SONAR_FAILSAVE_RANGE - sonar_values_filtered.right;
	e_right_sum    = e_right_sum + e_right;

	float out_right = e_right * SONAR_FAILSAVE_P +
		((e_right - e_right_old) * SONAR_FAILSAVE_D / t_a) +
		SONAR_FAILSAVE_I * t_a * e_right_sum;

	e_right_old = e_right;
	if(sonar_values_filtered.right > 120)
	{
		out_right   = 0;
		e_right_sum = 0;
		e_right_old = e_right;
	}

	// Difference between back and front
	double roll = 0;
	if(sonar_values_filtered.left <= 120 || sonar_values_filtered.right <= 120)
	{
		roll = out_left - out_right;
	}

	return roll;
}
