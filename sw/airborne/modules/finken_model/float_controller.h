#ifndef SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_
#define SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_

#include "finken_model_pid.h"
#include "modules/finken_model/finken_model_system.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

extern void float_controller_init(void);
extern void float_controller_periodic(void);
extern void send_float_pid_telemetry(struct transport_tx *trans, struct link_device *link);
#endif /* SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_ */
