/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias Münch
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

#include "modules/sonar/sonar_i2c.h"
#include "generated/airframe.h"
#include "mcu_periph/i2c.h"
#include "state.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

/** Sonar offset.
 *  Offset value in m (float)
 *  //equals to the height when the ADC gives 0
 *  distance mesured by the i2c sensor
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0.
#endif

/** Sonar scale.
 *  Sensor sensitivity in m/i2c (float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 1.0000
#endif

#ifndef SONAR_ADDR
#define SONAR_ADDR 0x71
#endif
#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c2
#endif

uint16_t sonar_meas;
bool_t sonar_data_available;
float sonar_distance;
float sonar_offset;
float sonar_scale;
uint8_t sonar_status;
#define SONAR_STATUS_IDLE 0
#define SONAR_STATUS_PENDING 1

struct i2c_transaction sonar_i2c_trans;
struct i2c_transaction sonar_i2c_trans2;

void sonar_i2c_init(void) {
  sonar_meas = 0;
  sonar_data_available = FALSE;
  sonar_distance = 0;
  sonar_offset = SONAR_OFFSET;
  sonar_scale = SONAR_SCALE;

  sonar_i2c_trans.status = I2CTransDone;
  sonar_i2c_trans2.status = I2CTransDone;
}

/** Read I2C value to update sonar measurement and request new value
 */
void sonar_read_periodic(void) {
#ifndef SITL
  if (sonar_i2c_trans2.status == I2CTransDone && sonar_status == SONAR_STATUS_IDLE) {
		//send 0x51 to sensor
		sonar_i2c_trans2.buf[0] = 0x51;
		i2c_transmit(&SONAR_I2C_DEV, &sonar_i2c_trans2, (SONAR_ADDR << 1) | 1, 1); // 7-Bit Adress + write Bit
	}
  if (sonar_i2c_trans.status == I2CTransDone && sonar_status == SONAR_STATUS_PENDING) {
    if(i2c_receive(&SONAR_I2C_DEV, &sonar_i2c_trans, SONAR_ADDR << 1, 2)) {
			sonar_data_available = TRUE;
			sonar_meas = ((uint16_t)(sonar_i2c_trans.buf[1]) << 8) | (uint16_t)(sonar_i2c_trans.buf[0]);	// recieve mesuarment
			// send read-command 0x51
			sonar_distance = (float)sonar_meas * sonar_scale + sonar_offset;
		}
	}
	sonar_status++;
	sonar_status %= 2;
  sonar_i2c_trans.status = I2CTransDone;
  sonar_i2c_trans2.status = I2CTransDone;

#else // SITL
#warn "Compiling without SITL"
  sonar_distance = stateGetPositionEnu_f()->z;
  Bound(sonar_distance, 0.1f, 7.0f);
#endif // SITL
#ifdef SENSOR_SYNC_SEND_SONAR
	sonar_meas = (uint16_t) sonar_status;
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_meas, &sonar_distance);
#endif
}

void sonar_read_event( void ) {
#ifndef SITL
#endif
}
