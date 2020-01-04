/*
 * encoder.c
 *
 *  Created on: 6 ���. 2017 �.
 *      Author: kulish_y
 */


#include "encoder.h"

void encoder_init(ENC_INIT *enc, uint16_t period)
{
	rcc_periph_clock_enable(enc->rcc_tim);
	rcc_periph_reset_pulse(enc->rst_tim);

	gpio_mode_setup(enc->enc_ch_a.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, enc->enc_ch_a.pins);
	gpio_set_af(enc->enc_ch_a.port, enc->enc_ch_a.alt_func, enc->enc_ch_a.pins);

	gpio_mode_setup(enc->enc_ch_b.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, enc->enc_ch_b.pins);
	gpio_set_af(enc->enc_ch_b.port, enc->enc_ch_b.alt_func, enc->enc_ch_b.pins);

	timer_set_oc_polarity_low(enc->timer, enc->oc_a);
	timer_set_oc_polarity_low(enc->timer, enc->oc_b);

	timer_slave_set_mode(enc->timer, TIM_SMCR_SMS_EM3); // encoder
	timer_ic_set_filter(enc->timer, enc->ic_a,TIM_IC_DTF_DIV_32_N_8);
	timer_ic_set_filter(enc->timer, enc->ic_b,TIM_IC_DTF_DIV_32_N_8);

	timer_ic_set_input(enc->timer, enc->ic_a, enc->ic_in_a);
	timer_ic_set_input(enc->timer, enc->ic_b, enc->ic_in_b);

	timer_set_period(enc->timer, period);
	timer_enable_counter(enc->timer);
}

