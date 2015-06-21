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
void setMinMax(float minParam, float maxParam, struct pid_controller *con)
{
	con->min = minParam;
	con->max = maxParam;
	con->checkMinMax = 1;
}

/*
 * This is the main method of the pid_controller. the error should be the distance to the desired distance.
 */
float adjust(float error, float time_step, struct pid_controller *con)
{
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

	return res;
}
