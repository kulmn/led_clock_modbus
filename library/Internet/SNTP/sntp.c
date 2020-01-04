/*
 * sntp.c
 *
 *  Created on: 2014. 12. 15.
 *      Author: Administrator
 */


#include <string.h>

#include "sntp.h"
#include "socket.h"


#define startindex  40 		  //last 8-byte of data_buf[size is 48 byte] is xmt, so the startindex should be 40

ntpformat NTPformat;
uint8_t ntpmessage[48];
uint8_t *data_buf;
uint8_t NTP_SOCKET;

uint16_t ntp_retry_cnt=0; //counting the ntp retry number



void SNTP_init(uint8_t s, uint8_t *ntp_server, uint8_t *buf)
{
	NTP_SOCKET = s;

	NTPformat.dstaddr[0] = ntp_server[0];
	NTPformat.dstaddr[1] = ntp_server[1];
	NTPformat.dstaddr[2] = ntp_server[2];
	NTPformat.dstaddr[3] = ntp_server[3];

	data_buf = buf;

	uint8_t Flag;
	NTPformat.leap = 0;           /* leap indicator */
	NTPformat.version = 4;        /* version number */
	NTPformat.mode = 3;           /* mode */
	NTPformat.stratum = 0;        /* stratum */
	NTPformat.poll = 0;           /* poll interval */
	NTPformat.precision = 0;      /* precision */
	NTPformat.rootdelay = 0;      /* root delay */
	NTPformat.rootdisp = 0;       /* root dispersion */
	NTPformat.refid = 0;          /* reference ID */
	NTPformat.reftime = 0;        /* reference time */
	NTPformat.org = 0;            /* origin timestamp */
	NTPformat.rec = 0;            /* receive timestamp */
	NTPformat.xmt = 1;            /* transmit timestamp */

	Flag = (NTPformat.leap<<6)+(NTPformat.version<<3)+NTPformat.mode; //one byte Flag
	memcpy(ntpmessage,(void const*)(&Flag),1);
}

int8_t SNTP_run(uint32_t *utc_seconds)
{
	uint16_t RSR_len;
	uint32_t destip = 0, seconds = 0;;
	uint16_t destport;

	switch(getSn_SR(NTP_SOCKET))
	{
	case SOCK_UDP:
		if ((RSR_len = getSn_RX_RSR(NTP_SOCKET)) > 0)
		{
			if (RSR_len > MAX_SNTP_BUF_SIZE) RSR_len = MAX_SNTP_BUF_SIZE;	// if Rx data size is lager than TX_RX_MAX_BUF_SIZE
			recvfrom(NTP_SOCKET, data_buf, RSR_len, (uint8_t *)&destip, &destport);

			for (uint8_t i = 0; i < 4; i++)
			{
				seconds = (seconds << 8) | data_buf[startindex + i];
			}

			*utc_seconds = seconds;

			ntp_retry_cnt=0;
			close(NTP_SOCKET);

			return 1;
		}
		sendto(NTP_SOCKET,ntpmessage,sizeof(ntpmessage),NTPformat.dstaddr,ntp_port);
		break;

	case SOCK_CLOSED:
		socket(NTP_SOCKET,Sn_MR_UDP,ntp_port,0);
		break;
	}
	// Return value
	// 0 - failed / 1 - success
	return 0;
}
