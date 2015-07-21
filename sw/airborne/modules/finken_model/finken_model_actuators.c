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
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"

struct actuators_model_s finken_actuators_model;
struct actuators_model_s finken_actuators_set_point;

void finken_actuators_model_init(void) {
	finken_actuators_model.alpha  = 0;
	finken_actuators_model.beta   = 0;
	finken_actuators_model.theta  = 0;
	finken_actuators_model.thrust = 0;

	register_periodic_telemetry(DefaultPeriodic, "FINKEN_ACTUATORS_MODEL", send_finken_actuators_model_telemetry);
}

void finken_actuators_model_periodic(void) {
	finken_actuators_model.alpha = finken_actuators_set_point.alpha;
	finken_actuators_model.beta = finken_actuators_set_point.beta;
	finken_actuators_model.theta = finken_actuators_set_point.theta;
	finken_actuators_model.thrust = compensate_battery_drop(finken_actuators_set_point.thrust);

}

float compensate_battery_drop(float thrust_setpoint) {
	float thrust = thrust_setpoint + (FINKEN_THRUST_LOW - FINKEN_THRUST_DEFAULT) * (84 - electrical.vsupply) / (84 - 65);
	if(thrust < 0.2)
		return 0.2;
	else if(thrust > 1.0)
		return 1.0;
	return thrust;
}
void send_finken_actuators_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_FINKEN_ACTUATORS_MODEL(
		DefaultChannel,
		DefaultDevice,
		&finken_actuators_model.alpha,
		&finken_actuators_model.beta,
		&finken_actuators_model.theta,
		&finken_actuators_model.thrust
	);
}
