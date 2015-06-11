/*
 * finken_model_pid.h
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#ifndef FINKEN_MODEL_PID_H_
#define FINKEN_MODEL_PID_H_

/*
 * Everything within this class is based on utilities_fun.lua, which should be linked in the wiki.
 */

struct pid_controller
{
	float integral = 0;			//i'm not quite sure how this works...
	float previousError = 0;	//the deviation from our desired distance at the last time step. Used to compare with current deviation.

	float min = 0;				//the controller has output boundaries.
	float max = 0;
	float checkMinMax = 0;		//not sure about this one. it doesn't look very important.

	float p,i,d;				//the parameters of the pid. Currently, "i" is always zero, so it should be called a pd_controller instead.
} ;

extern void newPIDController(float p, float i, float d);
extern void setMinMax(float minParam, float maxParam, struct pid_controller *con);
extern void adjust(float error, struct pid_controller *con);

#endif /* FINKEN_MODEL_PID_H_ */
