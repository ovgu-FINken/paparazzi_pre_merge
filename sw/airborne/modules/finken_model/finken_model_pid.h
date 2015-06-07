/*
 * finken_model_pid.h
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#ifndef FINKEN_MODEL_PID_H_
#define FINKEN_MODEL_PID_H_

struct pid_controller
{
	float integral = 0;
	float previousError = 0;

	float min = 0;
	float max = 0;
	float checkMinMax = 0;

	float p,i,d;
} ;

extern void newPIDController(float p, float i, float d);
extern void setMinMax(float minParam, float maxParam, struct pid_controller con);
extern void adjust(float error, struct pid_controller con);

#endif /* FINKEN_MODEL_PID_H_ */
