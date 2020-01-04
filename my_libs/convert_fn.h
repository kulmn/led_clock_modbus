/*
 * convert_fn.h
 *
 *  Created on: 14 ���� 2015 �.
 *      Author: kulish_y
 */

#ifndef CONVERT_FN_H_
#define CONVERT_FN_H_

#include <stdint.h>


#define _PORT(Prt,Pn)	(Prt)
#define _PIN(Prt,Pn)	(Pn)

#define PORT(PP)        _PORT(PP)
#define PIN(PP)         _PIN(PP)

typedef struct
{
  uint32_t 	port;
  uint16_t 	pins;
  uint8_t 	alt_func;
} GPIO_HW_PIN;


uint32_t uint32_to_bcd(uint32_t value);
uint8_t bcd_to_uint8(uint8_t bcd);
uint8_t *Uint32ToStr(uint32_t value, uint8_t *buffer);
uint8_t *int32_to_str(int32_t value, uint8_t *buffer);
uint16_t str_to_uint16( uint8_t *str);

uint16_t ComputeCRC16(const uint8_t * buf, uint16_t len);

void add_str2buf(uint8_t **buf, const uint8_t *str);



#endif /* CONVERT_FN_H_ */
