/*
 * finken_mode_pid.c
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#include "finken_model_pid.h"

/*
 * Maybe this method isn't even necessary. You could possibly create the struct directly where you want to have it.
 * Eitherways, this method returns a new pid_controller with default settings.
 */
struct pid_controller newPIDController(float p, float i, float d)
{
	struct pid_controller con;

	con.integral = 0;
	con.previousError = 0;

	con.min = 0;
	con.max = 0;
	con.checkMinMax = 0;

	con.p = p;
	con.i = i;
	con.d = d;

	return con;
}

/**
 * Set the min an max values of a pid_controller, which is passed as a parameter.
 */
void setMinMax(float minParam, float maxParam, struct pid_controller *con)
{
	con.min = minParam;
	con.max = maxParam;
	con.checkMinMax = 1;
}

/*
 * This is the main method of the pid_controller. the error should be the distance to the desired distance.
 */
void adjust(float error, struct pid_controller *con)
{
	float time_step = 0.1; 	//Todo this should be the duration since the last function call. I don't know how to retrieve this yet.

	con->integral = con->integral + (error * time_step);
	float derivative = (error - con->previousError) / time_step;

	if(con->previousError == 0)
	{
		derivative = 0;
	}

	con->previousError = error;

	float res = (con->p * error) + (con->i * con->integral) + (con->d * derivative);

	if(con->checkMinMax == 1)
	{
		if(res < con->min)
		{
			res = con->min;
		}
		else if( res > con->max)
		{
			res = con->max;
		}
	}
}
