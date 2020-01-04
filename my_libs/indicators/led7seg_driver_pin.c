/*
 * led_7_segm.c
 *
 *  Created on: 26 ���. 2017 �.
 *      Author: kulish_y
 */

#include "led7seg_driver_pin.h"


static void led7seg_driver_init(LED7SEG_Interface *interface, uint8_t digits_num)
{
	LED7SEG_Pin_Driver *driver = (LED7SEG_Pin_Driver*) interface;

	for(uint8_t i = 0; i < digits_num; i++)
		gpio_mode_setup(driver->digit[i].port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, driver->digit[i].pins);

	for(uint8_t i = 0; i < LED7SEG_SEGS_AMOUNT; i++)
		gpio_mode_setup(driver->seg_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, driver->seg_masks[i]);
}



static void led7seg_on_dig(LED7SEG_Interface *interface, uint8_t dig)
{
	LED7SEG_Pin_Driver *driver = (LED7SEG_Pin_Driver*) interface;
	gpio_set(driver->digit[dig].port, driver->digit[dig].pins);
}



static void led7seg_send(LED7SEG_Interface *interface, uint16_t data)
{
	LED7SEG_Pin_Driver *driver = (LED7SEG_Pin_Driver*) interface;

	uint16_t		set_data = 0, clr_data = 0;

	for(uint8_t i = 0; i < 8; i++)
	{
		if (data &  (1 << i) )	set_data |= driver->seg_masks[i];
		else		clr_data |= driver->seg_masks[i];
	}

	gpio_clear(driver->seg_port, clr_data);
	gpio_set(driver->seg_port, set_data);
}


static void led7seg_off_dig(LED7SEG_Interface *interface, uint8_t dig)
{
	LED7SEG_Pin_Driver *driver = (LED7SEG_Pin_Driver*) interface;
	gpio_clear(driver->digit[dig].port, driver->digit[dig].pins);
}


const LED7SEG_Interface LED7SEG_PIN_INTERFACE = { led7seg_driver_init, led7seg_send, led7seg_on_dig, led7seg_off_dig};

