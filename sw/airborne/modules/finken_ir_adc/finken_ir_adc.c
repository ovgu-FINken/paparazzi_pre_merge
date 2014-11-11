#include "modules/finken_ir_adc/finken_ir_adc.h"
#include "mcu_periph/adc.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/uart.h"
#include "messages.h"

uint16_t ir_measurement;
float ir_distance;
bool_t ir_data_available;

static struct adc_buf ir_adc;

void finken_ir_adc_init(void) {
    ir_measurement = 0;
    ir_data_available = FALSE;
    ir_distance = 0;

    adc_buf_channel(ADC_CHANNEL_IR, &ir_adc, DEFAULT_AV_NB_SAMPLE);

    register_periodic_telemetry(DefaultPeriodic, "FINKEN_IR_ADC", send_finken_ir_adc_telemetry);
}

void finken_ir_adc_periodic(void) {
    ir_measurement = ir_adc.sum / ir_adc.av_nb_sample;
    ir_data_available = TRUE;
    ir_distance = ir_measurement;
}

void send_finken_ir_adc_telemetry(void) {
  DOWNLINK_SEND_FINKEN_IR_ADC(DefaultChannel, DefaultDevice,
    &ir_distance
  );
}
