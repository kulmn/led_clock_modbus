

#include <timer_hal.h>

static tims_irq_record timers_irq[USE_TIMERS_NUM];

/******************************************************************************/
void timer_hl_init(TIMER_HL *timer_hl, uint32_t prescaler, uint32_t period, uint32_t interrupts)
{
	timer_set_prescaler(timer_hl->timer, prescaler );	// Set prescaler
	timer_set_period(timer_hl->timer, period );		// Set period
	// Set the timers global mode to:    - use no divider    - alignment edge     - count direction up
	timer_set_mode(timer_hl->timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP );
//	timer_enable_preload(timer_hl->timer );
	timer_continuous_mode(timer_hl->timer );
	timer_disable_irq(timer_hl->timer, 0xFFFF);
	timer_enable_irq(timer_hl->timer, interrupts );		// Enable interrupt or DMA
}

/******************************************************************************
oc_mode	TIM_OCM_PWM1, TIM_OCM_TOGGLE ...
/******************************************************************************/
void timer_hl_set_out(TIMER_HL *timer_hl, enum tim_oc_mode oc_mode)
{
	timer_disable_oc_output(timer_hl->timer, timer_hl->oc_id );
	timer_set_oc_mode(timer_hl->timer, timer_hl->oc_id, oc_mode );
	timer_set_oc_value(timer_hl->timer, timer_hl->oc_id, 0 );
	timer_enable_oc_output(timer_hl->timer, timer_hl->oc_id );

#ifdef STM32F0
	timer_set_repetition_counter(timer_hl->timer, 0 );
	TIM_BDTR(timer_hl->timer) |= TIM_BDTR_AOE;
#endif
}

/******************************************************************************/
void timer_hl_set_input(TIMER_HL *timer_hl, enum tim_ic_id ic, enum tim_ic_input in)
{
//	timer_enable_preload(timer_hl->timer );
	timer_ic_set_input(timer_hl->timer, ic, in);
	timer_ic_set_filter(timer_hl->timer, ic, TIM_IC_DTF_DIV_32_N_8);
	timer_ic_enable(timer_hl->timer, ic); 			// Capture enabled
}

/******************************************************************************/
void timer_hl_trig_init( TIMER_HL *timer_hl)
{
	timer_update_on_overflow(timer_hl->timer);		//  Only counter overflow generates an update interrupt or DMA request .
	timer_set_master_mode(timer_hl->timer, TIM_CR2_MMS_UPDATE);		// The update event is selected as a trigger output (TRGO).
}


/******************************************************************************/
void timer_hl_start(TIMER_HL *timer_hl)
{
     timer_enable_counter(timer_hl->timer);
     if (timer_hl->nvic_irqn)
     {
	     nvic_enable_irq(timer_hl->nvic_irqn);
     }
}

/******************************************************************************/
void timer_hl_stop(TIMER_HL *timer_hl)
{
     timer_disable_counter(timer_hl->timer);
     nvic_disable_irq(timer_hl->timer);
}


/******************************************************************************/
void timer_hl_set_irq_handler(TIMER_HL *timer_hl, void (*handler)(void *), void * handler_arg)
{
	switch (timer_hl->timer)
	{
#ifdef USE_TIMER1
		case TIM1:
			timers_irq[USE_TIM1].handler = handler;
			timers_irq[USE_TIM1].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER2
		case TIM2:
			timers_irq[USE_TIM2].handler = handler;
			timers_irq[USE_TIM2].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER3
			case TIM3:
			timers_irq[USE_TIM3].handler = handler;
			timers_irq[USE_TIM3].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER4
			case TIM4:
			timers_irq[USE_TIM4].handler = handler;
			timers_irq[USE_TIM4].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER6
		case TIM6:
			timers_irq[USE_TIM6].handler = handler;
			timers_irq[USE_TIM6].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER7
		case TIM7:
			timers_irq[USE_TIM7].handler = handler;
			timers_irq[USE_TIM7].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER9
			case TIM9:
			timers_irq[USE_TIM9].handler = handler;
			timers_irq[USE_TIM9].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER10
			case TIM10:
			timers_irq[USE_TIM10].handler = handler;
			timers_irq[USE_TIM10].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER11
			case TIM11:
			timers_irq[USE_TIM11].handler = handler;
			timers_irq[USE_TIM11].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER14
			case TIM14:
			timers_irq[USE_TIM14].handler = handler;
			timers_irq[USE_TIM14].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER15
			case TIM15:
			timers_irq[USE_TIM15].handler = handler;
			timers_irq[USE_TIM15].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER16
			case TIM16:
			timers_irq[USE_TIM16].handler = handler;
			timers_irq[USE_TIM16].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_TIMER17
			case TIM17:
			timers_irq[USE_TIM17].handler = handler;
			timers_irq[USE_TIM17].handler_arg = handler_arg;
			break;
#endif
	}

}

/******************************************************************************/

#ifdef USE_TIMER1
void TIM1_IRQHandler(void)
{
	timers_irq[USE_TIM1].handler( timers_irq[USE_TIM1].handler_arg );
}
#endif

#ifdef USE_TIMER2
void TIM2_IRQHandler(void)
{
	timers_irq[USE_TIM2].handler( timers_irq[USE_TIM2].handler_arg );
}
#endif

#ifdef USE_TIMER3
void TIM3_IRQHandler(void)
{
	timers_irq[USE_TIM3].handler( timers_irq[USE_TIM3].handler_arg );
}
#endif

#ifdef USE_TIMER4
void TIM4_IRQHandler(void)
{
	timers_irq[USE_TIM4].handler( timers_irq[USE_TIM4].handler_arg );
}
#endif

#ifdef USE_TIMER6
void TIM6_IRQHandler(void)
{
	timers_irq[USE_TIM6].handler( timers_irq[USE_TIM6].handler_arg );
}
#endif

#ifdef USE_TIMER7
void TIM7_IRQHandler(void)
{
	timers_irq[USE_TIM7].handler( timers_irq[USE_TIM7].handler_arg );
}
#endif

#ifdef USE_TIMER9
void TIM9_IRQHandler(void)
{
	timers_irq[USE_TIM9].handler( timers_irq[USE_TIM9].handler_arg );
}
#endif

#ifdef USE_TIMER10
void TIM10_IRQHandler(void)
{
	timers_irq[USE_TIM10].handler( timers_irq[USE_TIM10].handler_arg );
}
#endif

#ifdef USE_TIMER11
void TIM11_IRQHandler(void)
{
	timers_irq[USE_TIM11].handler( timers_irq[USE_TIM11].handler_arg );
}
#endif

#ifdef USE_TIMER14
void TIM14_IRQHandler(void)
{
	timers_irq[USE_TIM14].handler( timers_irq[USE_TIM14].handler_arg );
}
#endif

#ifdef USE_TIMER15
void TIM15_IRQHandler(void)
{
	timers_irq[USE_TIM15].handler( timers_irq[USE_TIM15].handler_arg );
}
#endif

#ifdef USE_TIMER16
void TIM11_IRQHandler(void)
{
	timers_irq[USE_TIM16].handler( timers_irq[USE_TIM16].handler_arg );
}
#endif

#ifdef USE_TIMER17
void TIM17_IRQHandler(void)
{
	timers_irq[USE_TIM17].handler( timers_irq[USE_TIM17].handler_arg );
}
#endif



