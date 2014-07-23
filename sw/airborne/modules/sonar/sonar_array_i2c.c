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

#include "modules/sonar/sonar_array_i2c.h"
#include "generated/airframe.h"
#include "mcu_periph/i2c.h"
#include "state.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"

#include <math.h>


/** Sonar offset.
 *  Offset value in m (float)
 *  distance mesured by the i2c sensor
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0.
#endif

/** Sonar scale.
 *  Scaling factor to compute real distances(float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 1.0000
#endif

#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c2
#endif

/** SONAR_ADDR_FRONT
 * 	adress for the front Sensor
 * 	same as RIGHT, LEFT, BACK ...
 */
#ifndef SONAR_ADDR_FRONT
#define SONAR_ADDR_FRONT 0x71
#endif

#ifndef SONAR_ADDR_RIGHT
#define SONAR_ADDR_RIGHT 0x72
#endif

#ifndef SONAR_ADDR_BACK
#define SONAR_ADDR_BACK 0x73
#endif

#ifndef SONAR_ADDR_LEFT
#define SONAR_ADDR_LEFT 0x74
#endif

#ifndef SONAR_ADDR_DOWN
#define SONAR_ADDR_DOWN 0x75
#endif

static const char read_order[] =  { SONAR_ADDR_FRONT, SONAR_ADDR_RIGHT, SONAR_ADDR_DOWN, SONAR_ADDR_BACK, SONAR_ADDR_LEFT};

float sonar_distance;
float sonar_fail_telemetry_pitch;
float sonar_fail_telemetry_roll;
uint8_t sonar_status;
uint8_t sonar_index;
struct sonar_values_s sonar_values;
struct sonar_values_s sonar_values_old;
struct sonar_data_available_s sonar_data_available;
struct sonar_values_s sonar_errors;
struct sonar_values_s sonar_errors_old;
#define SONAR_STATUS_IDLE 0
#define SONAR_STATUS_PENDING 1

struct i2c_transaction sonar_i2c_read_front_trans;
struct i2c_transaction sonar_i2c_read_right_trans;
struct i2c_transaction sonar_i2c_read_back_trans;
struct i2c_transaction sonar_i2c_read_left_trans;
struct i2c_transaction sonar_i2c_read_down_trans;
struct i2c_transaction sonar_i2c_write_trans;

void sonar_array_i2c_init(void) {
	sonar_index = 0;
	sonar_status = SONAR_STATUS_IDLE;

	sonar_i2c_read_front_trans.buf[0] = 0;
	sonar_i2c_read_front_trans.buf[1] = 0;
	sonar_i2c_read_right_trans.buf[0] = 0;
	sonar_i2c_read_right_trans.buf[1] = 0;
	sonar_i2c_read_back_trans.buf[0] = 0;
	sonar_i2c_read_back_trans.buf[1] = 0;
	sonar_i2c_read_left_trans.buf[0] = 0;
	sonar_i2c_read_left_trans.buf[1] = 0;
	sonar_i2c_read_down_trans.buf[0] = 0;
	sonar_i2c_read_down_trans.buf[1] = 0;

	sonar_data_available.front = FALSE;
	sonar_data_available.right = FALSE;
	sonar_data_available.back  = FALSE;
	sonar_data_available.left  = FALSE;
	sonar_data_available.down  = FALSE;

	sonar_values.front = 0;
	sonar_values.right = 0;
	sonar_values.back  = 0;
	sonar_values.left  = 0;
	sonar_values.down  = 0;

	sonar_errors.front = 0;
	sonar_errors.right = 0;
	sonar_errors.back  = 0;
	sonar_errors.left  = 0;
	sonar_errors.down  = 0;

	sonar_errors_old.front = 0;
	sonar_errors_old.right = 0;
	sonar_errors_old.back  = 0;
	sonar_errors_old.left  = 0;
	sonar_errors_old.down  = 0;
	// register telemetry
	register_periodic_telemetry(DefaultPeriodic, "SONAR_ARRAY", send_sonar_array_telemetry);
}


/** sonar_send_command
 *	send take_range_reading command (0x51) to the sonar sensors to trigger the range readin
 */
void sonar_send_command(uint8_t i2c_addr) {
	if (sonar_i2c_write_trans.status == I2CTransDone) {
		sonar_i2c_write_trans.buf[0] = 0x51;
		i2c_transmit(&SONAR_I2C_DEV, &sonar_i2c_write_trans, (i2c_addr << 1) & ~0x01, 1); // 7-Bit Adress + write Bit (last bit set to 0)
	}
	sonar_i2c_write_trans.status = I2CTransDone;
}


void query_sensor( int16_t* value, int16_t* old_value, uint8_t i2c_addr, struct i2c_transaction* transaction) {
	if(transaction->status == I2CTransDone) {
		i2c_receive(&SONAR_I2C_DEV, transaction, (i2c_addr << 1) | 1, 2);
		int16_t meas = (int16_t) (((uint16_t)(transaction->buf[0]) << 8) | (uint16_t)(transaction->buf[1]));	// recieve mesuarment

		if(meas > 0 && meas < 400) {
			*old_value = *value;
			*value = meas;
		} else {
			*value = *old_value;
		}
		
	}
	transaction->status = I2CTransDone;
}

void query_all_sensors( void ) {
#ifndef SITL
	query_sensor(&sonar_values.front,&sonar_values_old.front, SONAR_ADDR_FRONT, &sonar_i2c_read_front_trans);
	query_sensor(&sonar_values.right,&sonar_values_old.right, SONAR_ADDR_RIGHT, &sonar_i2c_read_right_trans);
	query_sensor(&sonar_values.back ,&sonar_values_old.back , SONAR_ADDR_BACK, &sonar_i2c_read_back_trans);
	query_sensor(&sonar_values.left ,&sonar_values_old.left , SONAR_ADDR_LEFT, &sonar_i2c_read_left_trans);
	query_sensor(&sonar_values.down ,&sonar_values_old.down , SONAR_ADDR_DOWN, &sonar_i2c_read_down_trans);
#endif
}

/** Read I2C value to update sonar measurement and request new value
*/
void sonar_array_i2c_periodic(void) {
#ifndef SITL
	sonar_index = (sonar_index + 1) % 5;
	sonar_send_command(read_order[sonar_index]);

	query_all_sensors();
#else // SITL
#warn "SITL not implemented for sonar_array_i2c yet"
#endif // SITL
}

#ifndef SONAR_FAILSAVE_RANGE
#define SONAR_FAILSAVE_RANGE 200
#endif

#ifndef SONAR_FAILSAVE_P
#define SONAR_FAILSAVE_P 0.4
#endif

#ifndef SONAR_FAILSAVE_D
#define SONAR_FAILSAVE_D 0.1
#endif

#ifndef SONAR_FAILSAVE_I
#define SONAR_FAILSAVE_I 0.0
#endif

/////// Throttel
#ifndef SONAR_FAILSAVE_P_DOWN
#define SONAR_FAILSAVE_P_DOWN 0.4
#endif

#ifndef SONAR_FAILSAVE_D_DOWN
#define SONAR_FAILSAVE_D_DOWN 0.1
#endif

#ifndef SONAR_FAILSAVE_I_DOWN
#define SONAR_FAILSAVE_I_DOWN 0.0
#endif

double e_front_sum=0;
double e_front_old=0;
double e_back_sum=0;
double e_back_old=0;

double e_left_sum=0;
double e_left_old=0;

double e_right_sum=0;
double e_right_old=0;

double e_down_sum=0;
double e_down_old=0;


float sonar_failsave_pitch( void ) {
	/*
		 if (sonar_values.front > sonar_values.back) {
		 float out = ( (SONAR_FAILSAVE_RANGE - sonar_values.back) * SONAR_FAILSAVE_P - (sonar_values_old.back - sonar_values.back) * SONAR_FAILSAVE_D);
		 if(out > 0)
		 return -out;
		 return 0.0;
		 } else {
		 float out = ( (SONAR_FAILSAVE_RANGE - sonar_values.front) * SONAR_FAILSAVE_P - (sonar_values_old.front - sonar_values.front) * SONAR_FAILSAVE_D);
		 if(out > 0)
		 return out;
		 return 0.0;
		 }

		 sonar_errors.front = SONAR_FAILSAVE_RANGE - sonar_values.front;
		 if(sonar_errors.front < 0) {
		 sonar_errors.front = 0;
		 }

		 sonar_errors.back = SONAR_FAILSAVE_RANGE - sonar_values.back;
		 if(sonar_errors.back < 0) {
		 sonar_errors.back = 0;
		 }

		 sonar_errors_old.front = SONAR_FAILSAVE_RANGE - sonar_values.front;
		 if(sonar_errors_old.front < 0) {
		 sonar_errors_old.front = 0;
		 }

		 sonar_errors_old.back = SONAR_FAILSAVE_RANGE - sonar_values.back;
		 if(sonar_errors_old.back < 0) {
		 sonar_errors_old.back = 0;
		 }

		 float e = sonar_errors.front - sonar_errors.back;
		 float de = e - sonar_errors_old.front - sonar_errors_old.back;

		 float y = e * SONAR_FAILSAVE_P - de * SONAR_FAILSAVE_D;
		 return y;
		 */

	// New PID control

	// Sonar Winkel korrektur
	//doule distanz_correct=0;
	//distanz_correct=cos(pitchangle)*sonar_values.front

	double saveDistanz=100;
	double Ta=0.166;

	// Distanz front
	double e_front= (saveDistanz - sonar_values.front);
	e_front_sum = e_front_sum + e_front;
	e_front_old = (saveDistanz - sonar_values_old.front);

	float out_front = e_front * SONAR_FAILSAVE_P +
		((e_front - e_front_old) * SONAR_FAILSAVE_D / Ta) + SONAR_FAILSAVE_I * Ta * e_front_sum;

	if(sonar_values.front>120)
	{
		out_front=0;
		e_front_sum=0;
	}

	// Distanz back
	double e_back= (saveDistanz - sonar_values.back);
	e_back_sum = e_back_sum + e_back;
	e_back_old = (saveDistanz - sonar_values_old.back);


	float out_back = e_back * SONAR_FAILSAVE_P +
		((e_back - e_back_old) * SONAR_FAILSAVE_D / Ta) + SONAR_FAILSAVE_I * Ta * e_back_sum;

	if(sonar_values.back>120)
	{
		out_back=0;
		e_back_sum=0;
	}

	// Differenz between back and front
	double pitchangle=0;
	if(sonar_values.front<=120 || sonar_values.back<=120)
	{
		pitchangle=out_front-out_back;
	}

	return pitchangle;
}

float sonar_throttel( void )
{
	double saveDistanz=100;
	double Ta=0.066;

	// Distanz down
	double e_down= (saveDistanz - sonar_values.down);
	e_down_sum = e_down_sum + e_down;

	float out_down = e_down * SONAR_FAILSAVE_P_DOWN +
		((e_down - e_down_old) * SONAR_FAILSAVE_D_DOWN / Ta) + SONAR_FAILSAVE_I_DOWN * Ta * e_down_sum;

	e_down_old= e_down;

	if(sonar_values.down<21)
	{
		out_down=0;
		e_down_sum=0;
		e_down_old=e_down;
	}


	float throttel=0;
	throttel=out_down+0.42;

	if(throttel>0.7)
	{
		return 0.7;
	}
	if(throttel<0.0)
	{
		return 0.0;
	}


	return throttel;

}


float sonar_failsave_roll( void ) {
	/*
		 if (sonar_values.left > sonar_values.right) {
		 float out = ( (SONAR_FAILSAVE_RANGE - 1.0 * sonar_values.right) * SONAR_FAILSAVE_P  - (sonar_values_old.right - sonar_values.right) * SONAR_FAILSAVE_D);
		 if(out > 0)
		 return -out;
		 return 0.0;
		 } else {
		 float out = ( (SONAR_FAILSAVE_RANGE - 1.0 * sonar_values.left) * SONAR_FAILSAVE_P - (sonar_values_old.left - sonar_values.left) * SONAR_FAILSAVE_D);
		 if(out > 0)
		 return out;
		 return 0.0;

		 sonar_errors.right = SONAR_FAILSAVE_RANGE - sonar_values.right;
		 if(sonar_errors.right < 0) {
		 sonar_errors.right = 0;
		 }

		 sonar_errors.left = SONAR_FAILSAVE_RANGE - sonar_values.left;
		 if(sonar_errors.left < 0) {
		 sonar_errors.left = 0;
		 }

		 sonar_errors_old.right = SONAR_FAILSAVE_RANGE - sonar_values.right;
		 if(sonar_errors_old.right < 0) {
		 sonar_errors_old.right = 0;
		 }

		 sonar_errors_old.left = SONAR_FAILSAVE_RANGE - sonar_values.left;
		 if(sonar_errors_old.left < 0) {
		 sonar_errors_old.left = 0;
		 }

		 float e = sonar_errors.right - sonar_errors.left;
		 float de = e - sonar_errors_old.right - sonar_errors_old.left;

		 float y = -1.0 * (e * SONAR_FAILSAVE_P - de * SONAR_FAILSAVE_D);
		 y=0;
		 return y;
		 */
	double saveDistanz=100;
	double Ta=0.05;

	// Distanz left
	double e_left= (saveDistanz - sonar_values.left);
	e_left_sum = e_left_sum + e_left;

	float out_left = e_left * SONAR_FAILSAVE_P +
		((e_left - e_left_old) * SONAR_FAILSAVE_D / Ta) + SONAR_FAILSAVE_I * Ta * e_left_sum;

	e_left_old= e_left;

	if(sonar_values.left>120)
	{
		out_left=0;
		e_left_sum=0;
		e_left_old=e_left;
	}

	// Distanz right
	double e_right= (saveDistanz - sonar_values.right);
	e_right_sum = e_right_sum + e_right;

	float out_right = e_right * SONAR_FAILSAVE_P +
		((e_right - e_right_old) * SONAR_FAILSAVE_D / Ta) + SONAR_FAILSAVE_I * Ta * e_right_sum;

	e_right_old= e_right;
	if(sonar_values.right>120)
	{
		out_right=0;
		e_right_sum=0;
		e_right_old=e_right;
	}

	// Differenz between back and front
	double roll=0;
	if(sonar_values.left<=120 || sonar_values.right<=120)
	{
		roll=out_left-out_right;
	}

	return roll;
}
void sonar_array_i2c_event( void ) {
	// i guess it it not possible to query verything so often
}

void send_sonar_array_telemetry(void) {
	sonar_fail_telemetry_pitch = sonar_failsave_pitch();
	sonar_fail_telemetry_roll = sonar_failsave_roll();
	DOWNLINK_SEND_SONAR_ARRAY(DefaultChannel, DefaultDevice,
			&sonar_values.front,
			&sonar_values.right,
			&sonar_values.back,
			&sonar_values.left,
			&sonar_values.down,
			&sonar_fail_telemetry_pitch,
			&sonar_fail_telemetry_roll);
}
