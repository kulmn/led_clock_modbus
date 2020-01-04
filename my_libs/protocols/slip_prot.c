/* slip Serial Line IP (SLIP) protocol implementation */

#include <protocols/slip_prot.h>
#include <string.h>  /* For memcpy() */

/* SLIP special character codes    */
#define END		0xC0    /* indicates end of packet */
#define ESC			0xDB    /* indicates byte stuffing */
#define ESC_END	0xDC    /* ESC ESC_END means END data byte */
#define ESC_ESC		0xDD    /* ESC ESC_ESC means ESC data byte */

/* SEND_PACKET: sends a packet of length "len", starting at location "p".  */
void slip_send_packet(SLIP *interface, uint8_t *p, uint8_t len)
{
	SLIP_Interface *driver = interface->driver;

	// send an initial END character to flush out any data
	// that may have accumulated in the receiver due to line noise
	driver->slip_send_byte(driver, END );
	while (len--)
	{
		switch (*p)
		{
			case END:
				driver->slip_send_byte(driver, ESC );
				driver->slip_send_byte(driver, ESC_END );
				break;

			case ESC:
				driver->slip_send_byte(driver, ESC );
				driver->slip_send_byte(driver, ESC_ESC );
				break;

			default:
				driver->slip_send_byte(driver, *p );
		}
		p++;
	}
	// tell the receiver that we're done sending the packet
	driver->slip_send_byte(driver, END );
}

/* RECV_PACKET: receives a packet into the buffer located at "p".
 *      If more than max_packet_size bytes are received, the packet will be truncated.
 *      Returns the number of bytes stored in the buffer.
 */
uint8_t slip_recv_packet(SLIP *interface, uint8_t *p)
{
	uint8_t c;

	SLIP_Interface *driver = interface->driver;

	// sit in a loop reading bytes until received bytes aviable
	while (!driver->slip_recv_byte(driver, &c ))// get a character to process
	{
		// if it's an END character then we're done with the packet
		if (c == END)
		{
			if (interface->bytes_received)
			{
				c = interface->bytes_received;
				interface->bytes_received = 0;
				return c;
			} else return 0;
		}

		if (c == ESC)
		{
			driver->slip_recv_byte(driver, &c );
			if (c == ESC_END) c = END;
			if (c == ESC_ESC) c = ESC;
		}
		if (interface->bytes_received < interface->max_packet_size) p[interface->bytes_received++] = c;
	}

	return 0;
}

