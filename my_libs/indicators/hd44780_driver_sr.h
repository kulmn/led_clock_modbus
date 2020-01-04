#ifndef HD44780_DRIVER_SR_H_
#define HD44780_DRIVER_SR_H_

#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include "hd44780.h"
#include "hc595.h"

typedef struct
{
	HD44780_Interface interface;
	HD44780_Mode addr_mode;
	uint16_t out_buf;
	HD44780_Pin dp_first;
	uint8_t dp_amount;
	HD44780_DelayMicrosecondsFn delay_us;
	uint16_t pin_mask[HD44780_PINS_AMOUNT];
} HD44780_SR_Driver;

extern const HD44780_Interface HD44780_INTERFACE;

#endif /* HD44780_DRIVER_SR_H_ */
