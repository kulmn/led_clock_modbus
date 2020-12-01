#include "main.h"


SemaphoreHandle_t xButtons_Smphr[4];
const GPIO_HW_PIN button_pins[4] = { {BUTTN_HOURS_UP}, {BUTTN_HOURS_DN}, {BUTTN_MINUTES_UP}, {BUTTN_MINUTES_DN} };

#define USART1_RX_BUF_SIZE	32
#define USART1_TX_BUF_SIZE	32

USART_HAL 			usart_1;
static uint8_t 		usart1_rx_buf[USART1_RX_BUF_SIZE];
static uint8_t 		usart1_tx_buf[USART1_TX_BUF_SIZE];

TIMER_HL 			delay_timer;
OWI 				t_int_sensor, t_ext_sensor;
OWI_Driver 			t_int_sensor_driver, t_ext_sensor_driver;
DS18B20_TypeDef 	int_cur_temp, ext_cur_temp;

LED7SEG			led_ind;
LED7SEG_SR_Driver	led_driver;
static uint8_t 		led7seg_buf[4];

DS1307				rtc_ds1307;
DS1307_DATA 		ds1307_data;

MODBUS			modbus_pr;
MB_Serial_Driver	mb_serial_drv;
static uint8_t 		mb_packet_buf[128];
extern uint16_t		usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint16_t		usRegInputBuf[REG_INPUT_NREGS];

uint16_t 			mb_time_in_munutes = 0, loc_time_in_munutes = 0;
uint16_t 			cur_temp=0;


/******************************************************************************/
void vGetTempTask(void *pvParameters)
{
	portBASE_TYPE xStatus;
	uint16_t temp = 0;
	float t_cur = 0;

	while (1)
	{
		DS18B20_Read_Struct(&t_int_sensor, &int_cur_temp );
		usRegInputBuf[0] = int_cur_temp.raw_value;

		DS18B20_Read_Struct(&t_ext_sensor, &ext_cur_temp );
		usRegInputBuf[1] = ext_cur_temp.raw_value;

		vTaskDelay(1000 );
	}
	vTaskDelete(NULL );
}

/******************************************************************************/
void vGetButtonStateTask (void *pvParameters)			// ~ ???  bytes of stack used
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;						// 1000/10ms = 100 Hz
	uint8_t buttons_cnt[] = {0,0,0,0,0,0};

	xLastWakeTime = xTaskGetTickCount();

	for (uint8_t i=0; i<4; i++) xButtons_Smphr[i] = xSemaphoreCreateCounting(10,0);

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		for (uint8_t i=0; i<4; i++)
		{
			if(!gpio_get(button_pins[i].port, button_pins[i].pins))
			{
				buttons_cnt[i]++;
				if( buttons_cnt[i] > 100) {buttons_cnt[i] = 80; xSemaphoreGive( xButtons_Smphr[i] ); }
				if( buttons_cnt[i] == 2) { xSemaphoreGive( xButtons_Smphr[i] ); }
			} else buttons_cnt[i] = 0;
		}

	}
	vTaskDelete(NULL);
}

/******************************************************************************/
void vModbusTask(void *pvParameters)
{
	for (;;)
	{
		/* Call the main polling loop of the Modbus protocol stack. */
		eMBPoll(&modbus_pr);
		vTaskDelay(10);
//		usart_send(USART1, 'S');
	}
	vTaskDelete(NULL);
}


/******************************************************************************/
void vIndDataOutTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = DATA_OUT_TASK_FRQ;		//  ms

	static uint16_t tcnt = 0, scnt = 0, dot_msk = 0;
	static uint8_t cur_displ  = DISPL_CLOCK;

	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency );

		DS1307_Get_All_Registers(&rtc_ds1307, &ds1307_data);

		uint8_t hours = bcd_to_uint8(ds1307_data.hours);
		uint8_t minutes = bcd_to_uint8(ds1307_data.minutes);
		uint8_t month = bcd_to_uint8(ds1307_data.month);
		uint8_t month_day = bcd_to_uint8(ds1307_data.month_day);
		uint8_t button_pressed = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			while (xSemaphoreTake( xButtons_Smphr[i], ( portTickType ) 0 ) == pdTRUE)
			{
				button_pressed = 1;
				switch (i)
				{
					case 0:
						if (cur_displ == DISPL_CLOCK)	if (hours < 23) hours++;
						if (cur_displ == DISPL_DATE)	if (month_day < 31) month_day++;
						break;
					case 1:
						if (cur_displ == DISPL_CLOCK)	if (hours > 0) hours--;
						if (cur_displ == DISPL_DATE)	if (month_day > 1) month_day--;
						break;
					case 2:
						if (cur_displ == DISPL_CLOCK)	if (minutes < 59) minutes++;
						if (cur_displ == DISPL_DATE)	if (month < 12) month++;
						break;
					case 3:
						if (cur_displ == DISPL_CLOCK)	if (minutes > 0) minutes--;
						if (cur_displ == DISPL_DATE)	if (month > 1) month--;
						break;
				}
			}
		}
		if (button_pressed)
		{
			scnt = BT_PRESS_DELAY_TIME; //s
			ds1307_data.hours = uint32_to_bcd(hours);
			ds1307_data.minutes = uint32_to_bcd(minutes);
			ds1307_data.month = uint32_to_bcd(month);
			ds1307_data.month_day = uint32_to_bcd(month_day);
			DS1307_Set_All_Registers(&rtc_ds1307, &ds1307_data);
		}

		// Delay after button pressed
		if (scnt > 0)	scnt--;
		else tcnt++;

#if USE_MODBUS > 0
		// update time from modbus master
		uint16_t new_mb_time_in_munutes = (usRegHoldingBuf[0] * 60) + usRegHoldingBuf[1];

		if (new_mb_time_in_munutes != mb_time_in_munutes)
		{
			mb_time_in_munutes = new_mb_time_in_munutes;
			loc_time_in_munutes = (bcd_to_uint8(ds1307_data.hours) * 60) + bcd_to_uint8(ds1307_data.minutes);

			int16_t diff = mb_time_in_munutes - loc_time_in_munutes;
			if ((diff > -100) && (diff < 100))
			{
				ds1307_data.hours = uint32_to_bcd(usRegHoldingBuf[0] );
				ds1307_data.minutes = uint32_to_bcd(usRegHoldingBuf[1] );
				DS1307_Set_All_Registers(&rtc_ds1307, &ds1307_data );
			}
		}
#endif


		// update indicator data
		if (tcnt < CLOCK_SHOW_TIME)			// s
		{
			cur_displ = DISPL_CLOCK;
		}else
			if (tcnt < PARAM1_SHOW_TIME)
			{
				#if SHOW_TEMP_EXT > 0
				cur_displ = DISPL_TEMP_EXT;
				#endif
				#if SHOW_DATE > 0
				cur_displ = DISPL_DATE;
				#endif
			}else
				if (tcnt < PARAM2_SHOW_TIME)
				{
					cur_displ = DISPL_TEMP_INT;
				}else tcnt = 0;


		switch (cur_displ)
		{
			case DISPL_CLOCK:
				if (dot_msk) dot_msk = 0;
				else dot_msk = 0x06;
				led7seg_write_two_bcd_bytes(&led_ind, ds1307_data.hours, ds1307_data.minutes, dot_msk );
				break;
			case DISPL_DATE:
				led7seg_write_two_bcd_bytes(&led_ind, ds1307_data.month_day, ds1307_data.month, 0x04 );
				break;
			case DISPL_TEMP_EXT:
				led7seg_write_ds18b20_temp(&led_ind, ext_cur_temp.value, ext_cur_temp.tens_value );
				break;
			case DISPL_TEMP_INT:
				led7seg_write_ds18b20_temp(&led_ind, int_cur_temp.value, ext_cur_temp.tens_value );
				break;
		}



	}

	vTaskDelete(NULL );
}



/******************************************************************************/
void vLed7segUpdateTask(void *pvParameters) 			//  ~ 21 * 4  bytes of stack used
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 3;		// 333 Hz

	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		led7seg_update(&led_ind);
	}
	vTaskDelete(NULL);
}


/******************************************************************************/
void init_led7seg(void)
{
	const GPIO_HW_PIN pins[LED7SEG_DIGITS_NUM] = { { LED_IND_DIG_0 }, { LED_IND_DIG_1 }, { LED_IND_DIG_2 }, { LED_IND_DIG_3 }  };

	led_ind.num_digits = 4;
	led_ind.cur_digit = 0;
	led_ind.buffer = (uint8_t *)led7seg_buf;
	led_ind.driver = (LED7SEG_Interface*) &led_driver;

	for (uint8_t i = 0; i < led_ind.num_digits; i++)
		led_driver.digit[i] = pins[i];

	led_driver.interface = LED7SEG_SR_INTERFACE;
	led_driver.seg_masks[LED7SEG_SEG_A] = LED_IND_SEG_A;
	led_driver.seg_masks[LED7SEG_SEG_B] = (LED_IND_SEG_B);
	led_driver.seg_masks[LED7SEG_SEG_C] = (LED_IND_SEG_C);
	led_driver.seg_masks[LED7SEG_SEG_D] =  (LED_IND_SEG_D);
	led_driver.seg_masks[LED7SEG_SEG_E] =  (LED_IND_SEG_E);
	led_driver.seg_masks[LED7SEG_SEG_F] =  (LED_IND_SEG_F);
	led_driver.seg_masks[LED7SEG_SEG_G] = (LED_IND_SEG_G);
	led_driver.seg_masks[LED7SEG_SEG_H] = (LED_IND_SEG_H);

	led7seg_init(&led_ind);
}


/******************************************************************************/
void init_modbus(void)
{

	USART_Init		init_data;

	rcc_periph_clock_enable(RCC_USART1);

	init_data.usart = USART1;
	init_data.baudrate= 9600;
	init_data.parity = USART_PARITY_NONE;
	init_data.usart_tx = (GPIO_HW_PIN) {USART1_TX_PIN};
	init_data.usart_rx = (GPIO_HW_PIN) {USART1_RX_PIN};
	init_data.rx_buf_ptr = (uint8_t *)usart1_rx_buf;
	init_data.tx_buf_ptr = (uint8_t *)usart1_tx_buf;
	init_data.rx_buf_size = USART1_RX_BUF_SIZE;
	init_data.tx_buf_size = USART1_TX_BUF_SIZE;
	uart_init(&usart_1, &init_data);

	usart_disable_rx_interrupt(usart_1.usart );
	usart_hl_set_irq_handler(&usart_1, MB_USART_ISR_FN, &modbus_pr);
	nvic_enable_irq(NVIC_USART1_IRQ);

	mb_serial_drv.interface = MODBUS_INTERFACE;
	mb_serial_drv.usart_hl = &usart_1;
	modbus_pr.driver = (Modbus_Interface*)&mb_serial_drv;
	modbus_pr.packet_buf = mb_packet_buf;
	eMBInit(&modbus_pr, 11 );

	usRegHoldingBuf[0] = 0;
	usRegHoldingBuf[1] = 0;

	eMBEnable(&modbus_pr);	// Enable the Modbus Protocol Stack.
}


/******************************************************************************/
void delay_timer_init(void)
{
	delay_timer.timer = TIM14;
//	timer_pwm1.oc_id = 0;		// not used
	delay_timer.nvic_irqn = 0;	// 0 - disable irq

	rcc_periph_clock_enable(RCC_TIM14);
	rcc_periph_reset_pulse(RST_TIM14);
	timer_hl_init( &delay_timer, (rcc_ahb_frequency/1000000)-1, UINT16_MAX, 0 ); 		// 1MHz (timer clock)
	timer_hl_start(&delay_timer);
}
void  delay_us(uint16_t delay)
{
	for(uint16_t start_cnt=TIM_CNT(delay_timer.timer); (uint16_t)(TIM_CNT(delay_timer.timer)-start_cnt) < delay; );
}
void  delay_ms(uint16_t delay)
{
	for(uint16_t i=0;i<delay;i++) delay_us(1000);
}



/******************************************************************************/
void init_owi(void)
{
	t_int_sensor_driver.interface=OWI_PINDRIVER_INTERFACE;
	t_int_sensor_driver.gpio = PORT(T_SENS_INT_PIN);
	t_int_sensor_driver.pin = PIN(T_SENS_INT_PIN);
	t_int_sensor.interface=(OWI_Interface*)&t_int_sensor_driver;
	t_int_sensor.delay_microseconds= delay_us;
	t_int_sensor.owi_device_n=T_SENS_INT_NUM;
	OWI_Init(&t_int_sensor);

	t_ext_sensor_driver.interface=OWI_PINDRIVER_INTERFACE;
	t_ext_sensor_driver.gpio = PORT(T_SENS_EXT_PIN);
	t_ext_sensor_driver.pin = PIN(T_SENS_EXT_PIN);
	t_ext_sensor.interface=(OWI_Interface*)&t_ext_sensor_driver;
	t_ext_sensor.delay_microseconds= delay_us;
	t_ext_sensor.owi_device_n=T_SENS_EXT_NUM;
	OWI_Init(&t_ext_sensor);

}

/******************************************************************************/
static void system_clock_setup(void)
{
	// Enable external high-speed oscillator 8MHz.
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);
//	rcc_set_sysclk_source(RCC_HSI);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

//	flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);
	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	// 8MHz * 4  = 32MHz
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL4);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 32000000;
	rcc_ahb_frequency = 32000000;
}


/******************************************************************************/
//----------------------------
// initializations
//----------------------------
void periphery_init()
{
	system_clock_setup();
	rcc_peripheral_enable_clock(&RCC_APB1ENR ,RCC_APB1ENR_PWREN);				// enable APB1 clocks

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
//	rcc_periph_clock_enable(RCC_GPIOC);
//	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOF);

	// Buttons init
	for (uint8_t i=0; i<4; i++)
		gpio_mode_setup(button_pins[i].port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, button_pins[i].pins);

	delay_timer_init();
	init_owi();
	DS18B20_Init(&t_int_sensor, DS18B20_12BIT_RES);
	DS18B20_Init(&t_ext_sensor, DS18B20_12BIT_RES);

	rtc_ds1307.SCL = (GPIO_HW_PIN) {I2C1_SCL};
	rtc_ds1307.SDA = (GPIO_HW_PIN) {I2C1_SDA};
	DS1307_Init(&rtc_ds1307, I2C1, RCC_I2C1, 8);

#if USE_MODBUS > 0
	init_modbus();
#endif


	init_led7seg();
}


void vApplicationIdleHook( void )
{

}

void vApplicationTickHook( void )
{

}

/******************************************************************************/
int main(void)
{
	periphery_init();
//	xTemperQueue=xQueueCreate( 5, sizeof( DS18B20_TypeDef ));
	xTaskCreate(vGetTempTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2 ,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vGetButtonStateTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(vIndDataOutTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vLed7segUpdateTask,(signed char*)"", configMINIMAL_STACK_SIZE * 1,	NULL, tskIDLE_PRIORITY + 2, NULL);

#if USE_MODBUS > 0
	xTaskCreate(vModbusTask,(signed char*)"", configMINIMAL_STACK_SIZE * 1,	NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

	vTaskStartScheduler();

	for( ;; );
}
