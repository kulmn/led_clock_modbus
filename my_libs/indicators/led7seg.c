/*
 * led7seg.c
 *
 *  Created on: 31 ���. 2017 �.
 *      Author: kulish_y
 */

#include "led7seg.h"

const uint8_t number[] =
{
	SEG_A_MASK | SEG_B_MASK | SEG_C_MASK | SEG_D_MASK | SEG_E_MASK | SEG_F_MASK, 				//0
	SEG_B_MASK | SEG_C_MASK, 																	//1
	SEG_A_MASK | SEG_B_MASK | SEG_D_MASK | SEG_E_MASK | SEG_G_MASK, 							//2
	SEG_A_MASK | SEG_B_MASK | SEG_C_MASK | SEG_D_MASK | SEG_G_MASK, 							//3
	SEG_B_MASK | SEG_C_MASK | SEG_F_MASK | SEG_G_MASK, 											//4
	SEG_A_MASK | SEG_C_MASK | SEG_D_MASK | SEG_F_MASK | SEG_G_MASK, 							//5
	SEG_A_MASK | SEG_C_MASK | SEG_D_MASK | SEG_E_MASK | SEG_F_MASK | SEG_G_MASK, 				//6
	SEG_A_MASK | SEG_B_MASK | SEG_C_MASK, 														//7
	SEG_A_MASK | SEG_B_MASK | SEG_C_MASK | SEG_D_MASK | SEG_E_MASK | SEG_F_MASK | SEG_G_MASK, 	//8
	SEG_A_MASK | SEG_B_MASK | SEG_C_MASK | SEG_D_MASK | SEG_F_MASK | SEG_G_MASK, 				//9
	SEG_A_MASK | SEG_D_MASK | SEG_E_MASK | SEG_F_MASK | SEG_G_MASK, 							//10 'E'
	SEG_E_MASK | SEG_G_MASK,					//11  'r'
};


void led7seg_init(LED7SEG *led_ind)
{
	led_ind->driver->led7seg_driver_init(led_ind->driver, led_ind->num_digits);
}


void led7seg_update(LED7SEG *led_ind)
{
	uint8_t	dig = led_ind->cur_digit;

	if (dig >= led_ind->num_digits) dig = 0;

	for(uint8_t i=0; i < led_ind->num_digits; i++ )	led_ind->driver->led7seg_off_dig(led_ind->driver, i);
	led_ind->driver->led7seg_send(led_ind->driver, led_ind->buffer[dig]);

	led_ind->driver->led7seg_on_dig(led_ind->driver, dig);

	dig++;
	led_ind->cur_digit = dig;

}


void led7seg_update_double(LED7SEG *led_ind, LED7SEG *led_ind2)
{
	uint8_t		dig = led_ind->cur_digit;
	uint16_t		tmp_data = 0;

	if (dig >= led_ind->num_digits) dig = 0;
	for(uint8_t i=0; i < led_ind->num_digits; i++ )	led_ind->driver->led7seg_off_dig(led_ind->driver, i);

	tmp_data = (led_ind2->buffer[dig] << 8) | led_ind->buffer[dig];
	led_ind->driver->led7seg_send(led_ind->driver, tmp_data);
	led_ind->driver->led7seg_on_dig(led_ind->driver, dig);

	dig++;
	led_ind->cur_digit = dig;

}


/******************************************************************************/
void led7seg_write_ds18b20_temp(LED7SEG *led_ind, int8_t value, uint8_t tens_value)
{
	uint16_t bcd_temper=0;

	for(uint8_t i=0; i < led_ind->num_digits; i++ )
		led_ind->buffer[i]  = 0;

	if (value < 0)
	{
		led_ind->buffer[3]=SEG_G_MASK;
		bcd_temper = uint32_to_bcd((~value)+1);

		if (value > -10)
		{
			led_ind->buffer[1] = number[tens_value];
			led_ind->buffer[2] = number[(bcd_temper & 0x000F)];
			led_ind->buffer[2] |= SEG_H_MASK;
		}else
		{
			led_ind->buffer[1] = number[(bcd_temper & 0x000F)];
			led_ind->buffer[2] = number[(bcd_temper & 0x00F0)>>4];
		}

	}else
	{
		bcd_temper = uint32_to_bcd(value);
		led_ind->buffer[1] = number[tens_value];
		led_ind->buffer[2] = number[(bcd_temper & 0x000F)];
		if ((bcd_temper & 0x00F0)>>4)	led_ind->buffer[3] = number[(bcd_temper & 0x00F0)>>4];
		else led_ind->buffer[3]=0;
		led_ind->buffer[2] |= SEG_H_MASK;
	}
	led_ind->buffer[0] |= SEG_H_MASK;
}



/******************************************************************************/
void led7seg_write_time(LED7SEG *led_ind, uint8_t bcd_hours, uint8_t bcd_minutes, uint8_t dots_mask)
{

	led_ind->buffer[3] = (led_ind->buffer[3] & SEG_H_MASK) | number[bcd_hours >> 4 ];
	led_ind->buffer[2] = (led_ind->buffer[2] & SEG_H_MASK) | number[bcd_hours & 0x0F ];
	led_ind->buffer[1] = (led_ind->buffer[1] & SEG_H_MASK) | number[bcd_minutes >> 4 ];
	led_ind->buffer[0] = (led_ind->buffer[0] & SEG_H_MASK) | number[bcd_minutes & 0x0F ];

	for(uint8_t i=0; i < led_ind->num_digits; i++ )
	{
		if (dots_mask & (1 << i)) led_ind->buffer[i] |= SEG_H_MASK;
		else led_ind->buffer[i] &= ~SEG_H_MASK;
	}
}


void led7seg_write_uint(LED7SEG *led_ind, uint16_t num)
{
	uint16_t		bcd_num;
	uint8_t		i, tmp, flg=0;

	bcd_num=uint32_to_bcd(num);

	/*
	i = led_ind->num_digits;
	while(i > 0)
	{
		if ( (bcd_num & (0x000F << (i<<2) )) == 0 ) led_ind->buffer[i] = 0;
		else cnt++;
		i--;
	}

	for(i = 0; i < led_ind->num_digits; i++)
	{
		led_ind->buffer[i] = number[ (bcd_num & (0x000F << (i<<2) ) ) >> (i<<2)];
	}
*/

	for(i = led_ind->num_digits; i >0 ; i--)
	{
		tmp = (bcd_num & (0x000F << ((i-1)<<2) ) ) >> ((i-1)<<2);
		if (tmp == 0 && flg==0 ) led_ind->buffer[(i-1)] = 0;
		else
		{
			flg=1;
			led_ind->buffer[(i-1)] = number[tmp];
		}


	}

}


void led7seg_write(LED7SEG *led_ind, uint8_t digit, uint8_t bcd_num)
{

	led_ind->buffer[digit] = number[bcd_num];
	/*
	for(uint8_t i = 0; i < 8; i++)
	{
		if (number[bcd_num] &  (1 << i) )
		{
			led_ind->buffer[digit] |= led_ind->seg_masks[i];
		}

	}
	*/
}

void led7seg_write_err(LED7SEG *led_ind)
{
	led_ind->buffer[0] = number[11];
	led_ind->buffer[1] = number[11];
	led_ind->buffer[2] = number[10];

}


void led7seg_off(LED7SEG *led_ind)
{
	for(uint8_t i=0; i < led_ind->num_digits; i++ )
	{
		led_ind->driver->led7seg_off_dig(led_ind->driver, i);
		led_ind->driver->led7seg_send(led_ind->driver, 0);
	}
}

