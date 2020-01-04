/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#ifndef _PORT_H
#define _PORT_H

#include <inttypes.h>


/* ----------------------- libopencm3 STM32 includes -------------------------------*/
#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/timer.h>

#include "FreeRTOS.h"

//#include "convert_fn.h"


/*

#define MB_USART				USART1
#define MB_USART_ISR			USART1_IRQHandler
#define MB_NVIC_USART_IRQ	NVIC_USART1_IRQ
#define MB_USART_RCC			RCC_USART1
#ifdef STM32F0
#define	USART_SR_TXE		USART_ISR_TXE
#define	USART_SR_RXNE		USART_ISR_RXNE
#endif



#define MB_TIMER				TIM16
#define MB_TIMER_RCC			RCC_TIM16
#define MB_TIMER_RST			RST_TIM16
#define MB_NVIC_TIMER_IRQ		NVIC_TIM16_IRQ
#define MB_TIMER_ISR			TIM16_IRQHandler
*/


#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

//#define ENTER_CRITICAL_SECTION( )   vMBPortEnterCritical()
//#define EXIT_CRITICAL_SECTION( )    vMBPortExitCritical()

#define ENTER_CRITICAL_SECTION( )   portENTER_CRITICAL()
#define EXIT_CRITICAL_SECTION( )    portEXIT_CRITICAL()

#define MB_PORT_HAS_CLOSE	                    1
//#define MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS    2






/* ----------------------- Prototypes ---------------------------------------*/
//void vMBPortEnterCritical( void );
//void vMBPortExitCritical( void );

#endif
