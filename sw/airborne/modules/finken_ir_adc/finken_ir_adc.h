#ifndef SONAR_ADC_H
#define SONAR_ADC_H

#include "std.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

extern uint16_t ir_measurement;
extern float ir_distance;
extern float ir_distance_close;
extern bool_t ir_data_available;

extern void finken_ir_adc_init(void);
extern void finken_ir_adc_periodic(void);

extern void send_finken_ir_adc_telemetry(struct transport_tx *trans, struct link_device* link);

#endif
