#include "float_controller.h"

struct pid_controller xFinkenFloatController;
struct pid_controller yFinkenFloatController;

float oldL, oldR, oldF, oldB;
float timeStep = 0.03;
float cap = 20;
void float_controller_init(void) {

	initFloatController(&xFinkenFloatController);
	initFloatController(&yFinkenFloatController);
}

void float_controller_periodic(void) {
	if (oldL != 0) {
		int xVelocity = getXDistanceDiff(); // / timeStep;
		int yVelocity = getYDistanceDiff(); // / timeStep;

		float xAcceleration = adjust(xVelocity, 0.03, &xFinkenFloatController);
		float yAcceleration = adjust(yVelocity, 0.03, &yFinkenFloatController);
		finken_actuators_set_point.alpha += xAcceleration;
		finken_actuators_set_point.beta += yAcceleration;
	}
	oldL = sonar_values.left;
	oldR = sonar_values.right;
	oldF = sonar_values.front;
	oldB = sonar_values.back;
}

int getXDistanceDiff() {
	if (sonar_values.front < sonar_values.back) {
		return sonar_values.front - oldF;
	} else {
		return -(sonar_values.back - oldB);
	}
}

int getYDistanceDiff() {
	if (sonar_values.left < sonar_values.right) {
		return sonar_values.left - oldF;
	} else {
		return -(sonar_values.right - oldB);
	}
}

void initFloatController(struct pid_controller *con) {
	con->p = 1;
	con->i = 0;
	con->d = 0;
	con->checkMinMax = 1;
	con->min = -cap;
	con->max = cap;
}
