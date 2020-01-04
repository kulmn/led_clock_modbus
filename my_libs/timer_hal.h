/*
 * timer_hal.h
 *
 *  Created on: 11 нояб. 2018 г.
 *      Author: yura
 */

#ifndef MY_LIBS_TIMER_HAL_H_
#define MY_LIBS_TIMER_HAL_H_

#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include "convert_fn.h"

#ifdef TIM1_BASE
//	#define USE_TIMER1
#endif
#ifdef TIM2_BASE
//	#define USE_TIMER2
#endif
#ifdef TIM3_BASE
//	#define USE_TIMER3
#endif
#ifdef TIM4_BASE
//	#define USE_TIMER4
#endif
#ifdef TIM6_BASE
//	#define USE_TIMER6
#endif
#ifdef TIM7_BASE
//	#define USE_TIMER7
#endif
#ifdef TIM9_BASE
//	#define USE_TIMER9
#endif
#ifdef TIM10_BASE
//	#define USE_TIMER10
#endif
#ifdef TIM11_BASE
//	#define USE_TIMER11
#endif
#ifdef TIM14_BASE
//	#define USE_TIMER14
#endif
#ifdef TIM15_BASE
//	#define USE_TIMER15
#endif
#ifdef TIM16_BASE
//	#define USE_TIMER16
#endif
#ifdef TIM17_BASE
//	#define USE_TIMER17
#endif


enum use_timers {
#ifdef USE_TIMER1
	USE_TIM1,
#endif
#ifdef USE_TIMER2
	USE_TIM2,
#endif
#ifdef USE_TIMER3
	USE_TIM3,
#endif
#ifdef USE_TIMER4
	USE_TIM4,
#endif
#ifdef USE_TIMER6
	USE_TIM6,
#endif
#ifdef USE_TIMER7
	USE_TIM7,
#endif
#ifdef USE_TIMER9
	USE_TIM9,
#endif
#ifdef USE_TIMER10
	USE_TIM10,
#endif
#ifdef USE_TIMER11
	USE_TIM11,
#endif
#ifdef USE_TIMER14
	USE_TIM14,
#endif
#ifdef USE_TIMER15
	USE_TIM15,
#endif
#ifdef USE_TIMER16
	USE_TIM16,
#endif
#ifdef USE_TIMER17
	USE_TIM17,
#endif
	USE_TIMERS_NUM,
};


typedef struct  {
void (*handler) (void *);
void * handler_arg;
}tims_irq_record;

typedef struct
{
	uint32_t				timer;
	enum tim_oc_id		oc_id;
//	enum tim_ic_id		ic_id;
	uint8_t 				nvic_irqn;
} TIMER_HL;


void timer_hl_init(TIMER_HL *timer_hl, uint32_t prescaler, uint32_t period, uint32_t interrupts);
void timer_hl_set_out(TIMER_HL *timer_hl, enum tim_oc_mode oc_mode);
void timer_hl_set_input(TIMER_HL *timer_hl, enum tim_ic_id ic, enum tim_ic_input in);
void timer_hl_trig_init( TIMER_HL *timer_hl);
void timer_hl_start(TIMER_HL *timer_hl);
void timer_hl_stop(TIMER_HL *timer_hl);
void timer_hl_set_irq_handler(TIMER_HL *timer_hl, void (*handler)(void *), void * handler_arg);

#endif /* MY_LIBS_TIMER_HAL_H_ */
