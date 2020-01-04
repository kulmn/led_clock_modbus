/*! \file buffer.c \brief Multipurpose byte buffer structure and methods. */
//*****************************************************************************
//
// File Name	: 'buffer.c'
// Title		: Multipurpose byte buffer structure and methods
// Author		: Pascal Stang - Copyright (C) 2001-2002
// Created		: 9/23/2001
// Revised		: 9/23/2001
// Version		: 1.0
// Target MCU	: any
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include "buffer.h"
//#include "global.h"
//#include "avr/io.h"

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START	uint8_t _sreg = SREG; cli()
#define CRITICAL_SECTION_END	SREG = _sreg
#endif

// global variables

// initialization

void bufferInit(cBuffer* buffer, uint8_t *start, uint16_t size)
{
	// begin critical section
//	CRITICAL_SECTION_START;
	// set start pointer of the buffer
	buffer->dataptr = start;
	buffer->size = size;
	// initialize index and length
	buffer->dataindex = 0;
	buffer->datalength = 0;
	// end critical section
//	CRITICAL_SECTION_END;
}

// access routines
uint8_t  bufferGetFromFront(cBuffer* buffer)
{
	uint8_t data = 0;
	// begin critical section
	CRITICAL_SECTION_START;
	// check to see if there's data in the buffer
	if(buffer->datalength)
	{
		// get the first character from buffer
		data = buffer->dataptr[buffer->dataindex];
		// move index down and decrement length
		buffer->dataindex++;
		if(buffer->dataindex >= buffer->size)
		{
			buffer->dataindex -= buffer->size;
		}
		buffer->datalength--;
	}
	// end critical section
	CRITICAL_SECTION_END;
	// return
	return data;
}

void bufferDumpFromFront(cBuffer* buffer, uint16_t numbytes)
{
	// begin critical section
	CRITICAL_SECTION_START;
	// dump numbytes from the front of the buffer
	// are we dumping less than the entire buffer?
	if(numbytes < buffer->datalength)
	{
		// move index down by numbytes and decrement length by numbytes
		buffer->dataindex += numbytes;
		if(buffer->dataindex >= buffer->size)
		{
			buffer->dataindex -= buffer->size;
		}
		buffer->datalength -= numbytes;
	}
	else
	{
		// flush the whole buffer
		buffer->datalength = 0;
	}
	// end critical section
	CRITICAL_SECTION_END;
}

uint8_t bufferGetAtIndex(cBuffer* buffer, uint16_t index)
{
	// begin critical section
	CRITICAL_SECTION_START;
	// return character at index in buffer
	uint8_t data = buffer->dataptr[(uint32_t) (buffer->dataindex+index) % (buffer->size)];
	// end critical section
	CRITICAL_SECTION_END;
	return data;
}

BUFFER_Result bufferAddToEnd(cBuffer* buffer, uint8_t data)
{
	// begin critical section
	CRITICAL_SECTION_START;
	// make sure the buffer has room
	if(buffer->datalength < buffer->size)
	{
		// save data byte at end of buffer
		buffer->dataptr[ (uint32_t)  (buffer->dataindex + buffer->datalength) % buffer->size] = data;
		// increment the length
		buffer->datalength++;
		// end critical section
		CRITICAL_SECTION_END;
		// return success
		return BUFFER_RESULT_OK;
	}
	// end critical section
	CRITICAL_SECTION_END;
	// return failure
	return BUFFER_RESULT_ERROR;
}

uint16_t bufferIsNotFull(cBuffer* buffer)
{
	// begin critical section
	CRITICAL_SECTION_START;
	// check to see if the buffer has room
	// return true if there is room
	uint16_t bytesleft = (buffer->size - buffer->datalength);
	// end critical section
	CRITICAL_SECTION_END;
	return bytesleft;
}

void bufferFlush(cBuffer* buffer)
{
	// begin critical section
	CRITICAL_SECTION_START;
	// flush contents of the buffer
	buffer->datalength = 0;
	// end critical section
	CRITICAL_SECTION_END;
}

