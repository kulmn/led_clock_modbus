#ifndef HD44780_DRIVER_PIN_H_
#define HD44780_DRIVER_PIN_H_

#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include "hd44780.h"
#include "convert_fn.h"


typedef struct
{
	HD44780_Interface interface;
	HD44780_Mode addr_mode;
	HD44780_Pin dp_first;
	uint8_t dp_amount;
	HD44780_DelayMicrosecondsFn delay_us;
	GPIO_HW_PIN pinout[HD44780_PINS_AMOUNT];
} HD44780_Pin_Driver;

extern const HD44780_Interface HD44780_PIN_INTERFACE;

#endif /* HD44780_DRIVER_PIN_H_ */
