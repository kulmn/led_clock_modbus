#ifndef __SLIPDEV_H__
#define __SLIPDEV_H__

#include <stdint.h>

//#define max_packet_size 64

struct SLIP_Interface;
typedef struct SLIP_Driver SLIP_Interface;

typedef enum
{
	SLIP_RESULT_OK, SLIP_RESULT_ERROR
} SLIP_Result;

/* Hardware abstraction layer */
struct SLIP_Driver
{
	SLIP_Result (*slip_send_byte)(SLIP_Interface *interface, uint8_t data);
	SLIP_Result (*slip_recv_byte)(SLIP_Interface *interface, uint8_t *data);
};

/* SLIP control structure */
typedef struct
{
	SLIP_Interface *driver;
	uint8_t bytes_received;
	uint8_t max_packet_size;
} SLIP;

void slip_send_packet(SLIP *interface, uint8_t *p, uint8_t len);
uint8_t slip_recv_packet(SLIP *interface, uint8_t *p);

#endif /* __SLIPDEV_H__ */

/** @} */
