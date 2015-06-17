#include "modules/finken_ir_adc/finken_ir_adc.h"
#include "mcu_periph/adc.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "messages.h"


#define IR_FAR_SAMPLE_SIZE 6
// adc-values to distance (in meters!)
static const uint16_t ir_in_samples_far[IR_FAR_SAMPLE_SIZE] = {800, 830, 900, 1150, 1650, 2700};
static const float ir_out_samples_far[IR_FAR_SAMPLE_SIZE] = {1.30, 1.00, .80, .60, .40, .20};

#define IR_CLOSE_SAMPLE_SIZE 3
// adc-values to distance (in meters!)
static const uint16_t ir_in_samples_close[IR_CLOSE_SAMPLE_SIZE] = {0, 2300, 2700};
static const float ir_out_samples_close[IR_CLOSE_SAMPLE_SIZE] = {0.0, .10, .20};

uint16_t ir_measurement;
float ir_distance;
float ir_distance_close;
bool_t ir_data_available;

static struct adc_buf ir_adc;

static float finken_ir_adc_far(uint16_t value){
		if(value <= ir_in_samples_far[0])
        return ir_out_samples_far[0];

    if(value >= ir_in_samples_far[IR_FAR_SAMPLE_SIZE - 1])
        return ir_out_samples_far[IR_FAR_SAMPLE_SIZE - 1];

		int i;
    for(i = 1; value > ir_in_samples_far[i]; i++);

    // w: something in range [0;1], for position in interval between i - 1 and i
    float w = (value - ir_in_samples_far[i - 1]) / (ir_in_samples_far[i] - ir_in_samples_far[i - 1]);
    return (1.0 - w) * ir_out_samples_far[i - 1] + w * ir_out_samples_far[i];
}

static float finken_ir_adc_close(uint16_t value){ 

		if(value <= ir_in_samples_close[0])
        return ir_out_samples_close[0];

    if(value >= ir_in_samples_close[IR_CLOSE_SAMPLE_SIZE - 1])
        return ir_out_samples_close[IR_CLOSE_SAMPLE_SIZE - 1];

		int i;
    for(i = 1; value > ir_in_samples_close[i]; i++);

    // w: something in range [0;1], for position in interval between i - 1 and i
    float w = (value - ir_in_samples_close[i - 1]) / (ir_in_samples_close[i] - ir_in_samples_close[i - 1]);
    return (1.0 - w) * ir_out_samples_close[i - 1] + w * ir_out_samples_close[i];
}

void finken_ir_adc_init(void)
{
    ir_measurement    = 0;
    ir_data_available = FALSE;
    ir_distance       = finken_ir_adc_far(ir_measurement);
    ir_distance_close = finken_ir_adc_close(ir_measurement);

    adc_buf_channel(ADC_CHANNEL_IR, &ir_adc, DEFAULT_AV_NB_SAMPLE);

    register_periodic_telemetry(DefaultPeriodic, "FINKEN_IR_ADC", send_finken_ir_adc_telemetry);
}

void finken_ir_adc_periodic(void) {
    ir_measurement    = ir_adc.sum / ir_adc.av_nb_sample;
		ir_distance       = finken_ir_adc_far(ir_measurement);
		ir_distance_close = finken_ir_adc_close(ir_measurement);
    ir_data_available = TRUE;
}

void send_finken_ir_adc_telemetry(struct transport_tx *trans, struct link_device* link)
{
    trans=trans;
    link=link;
    DOWNLINK_SEND_FINKEN_IR_ADC(DefaultChannel, DefaultDevice,
        &ir_distance,
				&ir_distance_close,
        &ir_measurement,
				&ir_data_available
    );
}
