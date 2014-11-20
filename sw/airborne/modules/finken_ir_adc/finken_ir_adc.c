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

#define IR_SAMPLE_SIZE 5
static uint16_t ir_in_samples[IR_SAMPLE_SIZE] = {1,2,3,4,5};
static float		ir_out_samples[IR_SAMPLE_SIZE] = {0.1,0.2,0.3,0.4,0.5};

void update_ir_distance_from_measurement(void) {
	if(ir_measurement <= ir_in_samples[0]) {
		ir_distance = ir_out_samples[0];
		return;
	}
	if(ir_measurement >= ir_in_samples[IR_SAMPLE_SIZE - 1]) {
		ir_distance = ir_out_samples[IR_SAMPLE_SIZE - 1];
		return;
	}
	int i = 0;
	while(ir_measurement <= ir_in_samples[++i]) {
		void;
	}
	float x = (float) (ir_measurement - ir_in_samples[i - 1]) / ir_in_samples[i];
	ir_distance = ir_out_samples[i - 1] + x * (ir_out_samples[i] - ir_out_samples[i - 1]);
}

void finken_ir_adc_periodic(void) {
    ir_measurement = ir_adc.sum / ir_adc.av_nb_sample;
    ir_data_available = TRUE;
		update_ir_distance_from_measurement();
}

void send_finken_ir_adc_telemetry(void) {
  DOWNLINK_SEND_FINKEN_IR_ADC(DefaultChannel, DefaultDevice,
    &ir_distance,
    &ir_measurement
  );
}
