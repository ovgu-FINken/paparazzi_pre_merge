#include "float_controller.h"

struct pid_controller xFinkenFloatController;
struct pid_controller yFinkenFloatController;

float oldL, oldR, oldF, oldB;
float timeStep = 0.03;
float cap = 15;
void float_controller_init(void) {
	initFloatController(&xFinkenFloatController);
	initFloatController(&yFinkenFloatController);
}

void float_controller_periodic(void) {
	if (oldL != 0) {
//		if (sonar_values.front != 765) {
//			int xVelocity = getXDistanceDiff(); // / timeStep;
//			float xAcceleration = adjust(xVelocity, 1, &xFinkenFloatController);
//			finken_actuators_set_point.alpha = xAcceleration;
//		}
//		if (sonar_values.left != 765) {
//			int yVelocity = getYDistanceDiff(); // / timeStep;
//			float yAcceleration = adjust(yVelocity, 1, &yFinkenFloatController);
//			finken_actuators_set_point.beta = -yAcceleration;
//		}
	}
	if (sonar_values.left != 765) {
		oldL = sonar_values.left;
	}
	if (sonar_values.front != 765) {
		oldR = sonar_values.right;
		oldF = sonar_values.front;
		oldB = sonar_values.back;
	}
}

int getXDistanceDiff() {
//	if (sonar_values.front < sonar_values.back) {
	return sonar_values.front - oldF;
//	} else {
//		return -(sonar_values.back - oldB);
//	}
}

int getYDistanceDiff() {
//	if (sonar_values.left < sonar_values.right) {
	return sonar_values.left - oldL;
//	} else {
//		return -(sonar_values.right - oldB);
//	}
}

