#include "float_controller.h"

struct pid_controller xPIDController;
struct pid_controller yPIDController;

float oldL, oldR, oldF, oldB;
float timeStep = 0.03;
int init = 0;

void float_controller_init(void) {
	initFloatController(&xPIDController);
	initFloatController(&yPIDController);
	register_periodic_telemetry(DefaultPeriodic, "FLOAT_PID", send_float_pid_telemetry);
}

void float_controller_periodic(void) {
	if (init) {
		int xVelocity = getXDistanceDiff() / timeStep;
		int yVelocity = getYDistanceDiff() / timeStep;

		float xAcceleration = adjust(-xVelocity, 0.03, &xPIDController);
		float yAcceleration = adjust(-yVelocity, 0.03, &yPIDController);
		finken_actuators_set_point.alpha += xAcceleration;
		finken_actuators_set_point.beta += yAcceleration;
	}
	init = 1;
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
	con->p = 0;
	con->i = 0.9;
	con->d = 0.1;
	con->checkMinMax = 0;
	float cap = 12;
	con->min = -cap;
	con->max = cap;
}

void send_float_pid_telemetry(struct transport_tx *trans, struct link_device *link) {
trans = trans;
link = link;
DOWNLINK_SEND_X_PID(DefaultChannel, DefaultDevice, &xPIDController.t, &xPIDController.p, &xPIDController.i, &xPIDController.d, &xPIDController.previousError,
		&xPIDController.res);
}
