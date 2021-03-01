/*
 * led7seg.h
 *
 *  Created on: 31 ���. 2017 �.
 *      Author: kulish_y
 */

#ifndef MY_LIBS_INDICATORS_LED7SEG_H_
#define MY_LIBS_INDICATORS_LED7SEG_H_

#include <stdint.h>
#include "convert_fn.h"

struct LED7SEG_Driver;
typedef struct LED7SEG_Driver LED7SEG_Interface;

/* Hardware abstraction layer */
struct LED7SEG_Driver
{
	void (*led7seg_driver_init)(LED7SEG_Interface *interface, uint8_t digits_num);
	void (*led7seg_send)(LED7SEG_Interface *interface, uint16_t data);
	void (*led7seg_on_dig)(LED7SEG_Interface *interface, uint8_t digit);
	void (*led7seg_off_dig)(LED7SEG_Interface *interface, uint8_t digit);
};


typedef enum
{
  LED7SEG_PIN_DIG0,
  LED7SEG_PIN_DIG1,
  LED7SEG_PIN_DIG2,
  LED7SEG_PIN_DIG3,

  LED7SEG_PINS_AMOUNT // enum member counter, must be last
} LED7SEG_Digit;


typedef enum
{
	LED7SEG_SEG_A,
	LED7SEG_SEG_B,
	LED7SEG_SEG_C,
	LED7SEG_SEG_D,
	LED7SEG_SEG_E,
	LED7SEG_SEG_F,
	LED7SEG_SEG_G,
	LED7SEG_SEG_H,

	LED7SEG_SEGS_AMOUNT // enum member counter, must be last
} LED7SEG_Segm;


// LED Segments masks
#define SEG_A_MASK			( (uint8_t) 1 << LED7SEG_SEG_A )
#define SEG_B_MASK			( (uint8_t) 1 << LED7SEG_SEG_B )
#define SEG_C_MASK			( (uint8_t) 1 << LED7SEG_SEG_C )
#define SEG_D_MASK			( (uint8_t) 1 << LED7SEG_SEG_D )
#define SEG_E_MASK			( (uint8_t) 1 << LED7SEG_SEG_E )
#define SEG_F_MASK			( (uint8_t) 1 << LED7SEG_SEG_F )
#define SEG_G_MASK			( (uint8_t) 1 << LED7SEG_SEG_G )
#define SEG_H_MASK			( (uint8_t) 1 << LED7SEG_SEG_H )


/* HD44780 control structure */
typedef struct
{
	LED7SEG_Interface *driver;
	uint8_t *buffer;
	uint8_t num_digits;
	uint8_t cur_digit;
} LED7SEG;

void led7seg_init(LED7SEG *led_ind);
void led7seg_update(LED7SEG *led_ind);
void led7seg_update_double(LED7SEG *led_ind, LED7SEG *led_ind2);
void led7seg_write_ds18b20_temp(LED7SEG *led_ind, int8_t value, uint8_t tens_value);
void led7seg_write_two_bcd_bytes(LED7SEG *led_ind, uint8_t bcd_hight, uint8_t bcd_low, uint8_t dots_mask);
void led7seg_write_uint(LED7SEG *led_ind, uint16_t num);
void led7seg_write(LED7SEG *led_ind, uint8_t digit, uint8_t bcd_num);
void led7seg_write_err(LED7SEG *led_ind);
void led7seg_off(LED7SEG *led_ind);

#endif /* MY_LIBS_INDICATORS_LED7SEG_H_ */
