/*
 * finken_mode_pid.c
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#include "finken_model_pid.h"

//struct pid_controller controller_;


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

void setMinMax(float minParam, float maxParam, struct pid_controller *con)
{
	con.min = minParam;
	con.max = maxParam;
	con.checkMinMax = 1;
}

//Todo minDist - dist
void adjust(float error, struct pid_controller *con)
{
	float time_step = getTimeStep(); 	//Todo irgendwie zeitlichen Abstand bestimmen

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
