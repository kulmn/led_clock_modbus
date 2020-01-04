
#include "../my_libs/convert_fn.h"


//const uint16_t  pow10Table16[]={ 10000, 1000, 100, 10, 1 };
const uint32_t  pow10Table32[]={1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1 };



/******************************************************************************
uint16_t uint16_to_bcd(uint16_t Value)
{
   uint32_t temp;

   temp = Value;
   for(uint8_t i = 0; i < 16; i++)
   {
	   if ((temp & 0xF0000) >= 0x50000) temp += 0x30000;
	   if ((temp & 0xF00000) >= 0x500000) temp += 0x300000;
	   if ((temp & 0xF000000) >= 0x5000000) temp += 0x3000000;
	   if ((temp & 0xF0000000) >= 0x50000000) temp += 0x30000000;
	   temp = temp<<1;
   }

   return (temp >> 16);
}

/******************************************************************************
uint16_t uint16_to_bcd(uint16_t value)
{
	uint32_t pow10;
	uint16_t bcd=0;
	uint8_t i,count;

   i = 0;				// value = 1 to 9999
   while(value < pow10Table32[ i ]) i++;
   while(i < 10)
   {
	  bcd = bcd << 4;
      pow10 = pow10Table32[ i++ ];
      count = 0;
      while(value >= pow10)
      {
         count ++;
         value -= pow10;
      }
      bcd |= count & 0x000F;
   }
   return bcd;
}

/******************************************************************************/
uint32_t uint32_to_bcd(uint32_t value)
{
	uint32_t pow10, bcd;
	uint8_t i,count;

   i = 0;
   bcd=0;

   if (value==0) return 0;

   while(value < pow10Table32[ i ]) i++;
   while(i < 10)
   {
	  bcd = bcd << 4;
      pow10 = pow10Table32[ i++ ];
      count = 0;
      while(value >= pow10)
      {
         count ++;
         value -= pow10;
      }
      bcd |= count & 0x000F;
   }
   return bcd;
}

/******************************************************************************/
uint8_t bcd_to_uint8(uint8_t bcd)
{
	uint8_t dec = 10 * (bcd >> 4);
	dec += bcd & 0x0F;
	return dec;
}


/******************************************************************************/
uint8_t *Uint32ToStr(uint32_t value, uint8_t *buffer)
{
	uint32_t pow10;
	uint8_t *ptr;
	uint8_t i,count;

	ptr = buffer;
	if(value == 0)
	{
		buffer[0] = '0';
		ptr++;
      buffer[1] = 0;
		return ptr;
	}

   i = 0;
   while(value < pow10Table32[i]) i++;
   while(i < 10)
   {
      pow10 = pow10Table32[i++];
      count = 0;
      while(value >= pow10)
      {
         count ++;
         value -= pow10;
      }
      *ptr++ = count + '0';
   }
   *ptr = 0;
   //return buffer;
   return ptr;
}

/******************************************************************************/
uint8_t *int32_to_str(int32_t value, uint8_t *buffer)
{
	uint8_t *ptr;

	ptr = buffer;
	if (value < 0)
	{
		value = (uint32_t)((~value)+1);
		*ptr++ = '-';
		ptr = Uint32ToStr(value, ptr);
	}else ptr = Uint32ToStr(value, buffer);
	return ptr;
}

/**
	converts a decimal ASCII characters to 16bit binary value
*/
uint16_t str_to_uint16( uint8_t *str)
{
    uint16_t res = 0;
    while(*str)
    {
        if( (*str >= '0') && (*str <= '9'))		res = (res * 10) + (*str - '0');
        str++;
    }
    return res;
}


uint16_t ComputeCRC16(const uint8_t * buf, uint16_t len)
{
	uint16_t crc = 0xffff; //seed
	for (uint16_t j = 0; j < len; j++)
	{
		uint8_t b = buf[j];
		for (uint8_t  i = 0; i < 8; i++)
		{
			crc = ((b ^ (uint8_t)crc) & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
			b >>= 1;
		}
	}
return crc;
}

/******************************************************************************/
void add_str2buf(uint8_t **buf, const uint8_t *str)
{
	while(*str) *((*buf)++) = *(str++);
	*(*buf) = 0;
}

