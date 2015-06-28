/*
 * finken_model_pid.c
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#include "finken_model_pid.h"

/**
 * Set the min an max values of a pid_controller, which is passed as a parameter.
 */
void setMinMax(float minParam, float maxParam, struct pid_controller *con) {
	con->min = minParam;
	con->max = maxParam;
	con->checkMinMax = 1;
}

/*
 * This is the main method of the pid_controller. the error should be the distance to the desired distance.
 */
float adjust(float error, float time_step, struct pid_controller *con) {
	con->t = time_step;
//	con->integral = con->integral + (error * time_step);
	float derivative = (error - con->previousError) / time_step;

	if (con->previousError == 0) {
		derivative = 0;
	}
	add_iPart(con, error * time_step);
	con->previousError = error;
	con->pPart = con->p * error;
	con->dPart = con->d * derivative;
	float res = con->pPart + con->iPart + con->dPart;

	if (con->checkMinMax == 1) {
		if (res < con->min) {
			res = con->min;
		} else if (res > con->max) {
			res = con->max;
		}
	}
	//res = -res;
	con->res = res;
	return res;
}

void initWallController(struct pid_controller *con) {
	con->p = 4.7;
	con->i = 0;
	con->d = 1;
	float cap = 0;
	con->min = -cap;
	con->max = cap;
	con->checkMinMax = 1;
	con->index = 0;
	con->k = 6;
	con->iPart = 0;
}

void initFloatController(struct pid_controller *con) {
	initWallController(con);
	con->p = 1;
	con->i = 0.0025;
	con->d = 0;
	con->checkMinMax = 1;
	con->min = -15;
	con->max = 15;
}

/*
 * This method allows to add a new i_error into the ringbuffer
 */
extern void add_iPart(struct pid_controller *con, float i_error) {
	con->index = con->index % con->k;
//	con->iPart = con->iPart - con->ringbuffer[con->index] + i_error;i
	float sum = 0;
	for (int i = 0; i < con->k; i++){
		sum += con->ringbuffer[i];
	}
	con->iPart = sum;
	con->ringbuffer[con->index] = i_error;
	con->index++;

//	Todo 1.) shouldn't the current i_error be added to the sum?
//	Todo 2.) maybe (iPart /= ringbuffer.size) ?
}

