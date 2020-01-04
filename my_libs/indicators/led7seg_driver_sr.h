/*
 * led_7_segm.h
 *
 *  Created on: 26 ���. 2017 �.
 *      Author: kulish_y
 */

#ifndef LED7SEG_DRIVER_SR_H_
#define LED7SEG_DRIVER_SR_H_

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
#include "led7seg.h"
#include "hc595.h"
#include "convert_fn.h"


#define LED7SEG_DIGITS_NUM 		4


typedef struct
{
	LED7SEG_Interface interface;
	uint32_t seg_port;
	uint16_t seg_masks[LED7SEG_SEGS_AMOUNT];
	GPIO_HW_PIN digit[LED7SEG_DIGITS_NUM];
} LED7SEG_SR_Driver;

extern const LED7SEG_Interface LED7SEG_SR_INTERFACE;


#endif /* LED7SEG_DRIVER_SR_H_ */
