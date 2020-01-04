/*
 * encoder.h
 *
 *  Created on: 6 ���. 2017 �.
 *      Author: kulish_y
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "convert_fn.h"



typedef struct
{
	enum rcc_periph_clken rcc_tim;
	enum rcc_periph_rst rst_tim;
	uint32_t			timer;

	GPIO_HW_PIN 	enc_ch_a;
	GPIO_HW_PIN 	enc_ch_b;

	enum tim_ic_id		ic_a;
	enum tim_ic_id		ic_b;
	enum tim_oc_id	oc_a;
	enum tim_oc_id	oc_b;

	enum tim_ic_input	ic_in_a;
	enum tim_ic_input	ic_in_b;

} ENC_INIT;


void encoder_init(ENC_INIT *enc, uint16_t period);

#endif /* ENCODER_H_ */
