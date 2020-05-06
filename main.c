/*
 * main.c
 *
 *  Created on: 08 сент. 2014 г.
 *      Author: yura
 */

#include "main.h"



//xQueueHandle xTemperQueue;
SemaphoreHandle_t xButtons_Smphr[4];
const GPIO_HW_PIN button_pins[4] = { {BUTTN_HOURS_UP}, {BUTTN_HOURS_DN}, {BUTTN_MINUTES_UP}, {BUTTN_MINUTES_DN} };

USART_HAL usart_1;

#define USART1_RX_BUF_SIZE	32
#define USART1_TX_BUF_SIZE	32

static uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE];
static uint8_t usart1_tx_buf[USART1_TX_BUF_SIZE];

TIMER_HL delay_timer;
OWI t_int_sensor, t_ext_sensor;
OWI_Driver t_int_sensor_driver, t_ext_sensor_driver;
DS18B20_TypeDef int_cur_temp, ext_cur_temp;


LED7SEG led_ind;
//LED7SEG_Pin_Driver led_driver;
LED7SEG_SR_Driver led_driver;
static uint8_t led7seg_buf[4];


DS1307		rtc_ds1307;
DS1307_DATA 	ds1307_data;
//SemaphoreHandle_t xDS1307_Mutex;


MODBUS			modbus_pr;
MB_Serial_Driver	mb_serial_drv;
static uint8_t 		mb_packet_buf[128];


extern uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint16_t   usRegInputBuf[REG_INPUT_NREGS];
uint16_t			mb_minutes; 	/*!< Minutes parameter, from 00 to 59 */
uint16_t 		mb_hours;   		/*!< Hours parameter, 24Hour mode, 00 to 23 */

uint16_t cur_temp=0;



void vGreenLedTask (void *pvParameters)
{
    while(1)
    {
//    	gpio_toggle(LED_GREEN);	/* LED on/off */
//    	usart_send_blocking(USART2,  '0');
    	vTaskDelay(500);
//    	gpio_clear(LED_GREEN);
//    	vTaskDelay(500);

     }
    vTaskDelete(NULL);
}

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

/*
		usart_hl_send_str(&usart_1, (uint8_t *)" int sensor:");
		usart_hl_send(&usart_1, int_cur_temp.value);
		usart_hl_send_str(&usart_1, (uint8_t *)".");
		usart_hl_send(&usart_1, int_cur_temp.tens_value);

		usart_hl_send_str(&usart_1, (uint8_t *)" ext sensor:");
		usart_hl_send(&usart_1, ext_cur_temp.value);
		usart_hl_send_str(&usart_1, (uint8_t *)".");
		usart_hl_send(&usart_1, ext_cur_temp.tens_value);


//		uint8_t clk_data = DS1307_GetReg( DS1307_MINUTES, 0);
		usart_hl_send_str(&usart_1, (uint8_t *)" ds1338: ");
//		usart_hl_send(&usart_1, clk_data);

		usart_hl_send(&usart_1, bcd_to_uint8(ds1307_data.minutes) );


//		uint8_t lcd_line[17];
//		rtc_hl_get_time_string(lcd_line, RTC_HL_HMS);
//		usart_hl_send_str(&usart_1, lcd_line);
*/


//		led7seg_write_ds18b20_temp(&led_ind, int_cur_temp.value, int_cur_temp.tens_value);

//    	xStatus = xQueueSendToBack( xTemperQueue, &cur_temper, 0 );
//    	if( xStatus != pdPASS )  { }

		vTaskDelay(1500 );
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
void vIndDataOutTask(void *pvParameters) //  ~ 21 * 4  bytes of stack used
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 500;		//  1\2 s

	static uint16_t tcnt = 0, dot_msk = 0;

	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency );

		DS1307_Get_All_Registers(&rtc_ds1307, &ds1307_data);

		uint8_t hours = bcd_to_uint8(ds1307_data.hours);
		uint8_t minutes = bcd_to_uint8(ds1307_data.minutes);
		uint8_t button_pressed = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			while (xSemaphoreTake( xButtons_Smphr[i], ( portTickType ) 0 ) == pdTRUE)
			{
				button_pressed = 1;
				switch (i)
				{
					case 0:
						if (hours < 23) hours++; break;
					case 1:
						if (hours > 0) hours--;	 break;
					case 2:
						if (minutes < 59) minutes++; break;
					case 3:
						if (minutes > 0) minutes--; break;
				}
			}
		}
		if (button_pressed)
		{
			tcnt = 10;
			ds1307_data.hours = uint32_to_bcd(hours);
			ds1307_data.minutes = uint32_to_bcd(minutes);
			DS1307_Set_All_Registers(&rtc_ds1307, &ds1307_data);
		}

		// update time from modbus master
		if ( (mb_minutes !=usRegHoldingBuf[0]) ||(mb_hours !=usRegHoldingBuf[1]) )
		{
			mb_minutes = usRegHoldingBuf[0];
			mb_hours = usRegHoldingBuf[1];
			ds1307_data.hours = uint32_to_bcd(mb_hours);
			ds1307_data.minutes = uint32_to_bcd(mb_minutes);
			DS1307_Set_All_Registers(&rtc_ds1307, &ds1307_data);
		}


		if (tcnt < 20)			// 10s
		{
			if (dot_msk)	dot_msk = 0;
			else dot_msk = 0x04;

			led7seg_write_time(&led_ind, ds1307_data.hours, ds1307_data.minutes, dot_msk);
		}else
			if (tcnt < 24)
			{
				led7seg_write_ds18b20_temp(&led_ind, ext_cur_temp.value, ext_cur_temp.tens_value);
			}else
				if (tcnt < 28)
				{
					led7seg_write_ds18b20_temp(&led_ind, int_cur_temp.value, ext_cur_temp.tens_value);
				}else tcnt = 0;

		tcnt++;
	}

	vTaskDelete(NULL );
}



/******************************************************************************/
void vLed7segUpdateTask(void *pvParameters) 			//  ~ 21 * 4  bytes of stack used
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 5;		// 200 Hz

	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		led7seg_update(&led_ind);
	}
	vTaskDelete(NULL);
}



void init_led7seg(void)
{
	const GPIO_HW_PIN pins[LED7SEG_DIGITS_NUM] = { { LED_IND_DIG_0 }, { LED_IND_DIG_1 }, { LED_IND_DIG_2 }, { LED_IND_DIG_3 }  };

	led_ind.num_digits = 4;
	led_ind.cur_digit = 0;
	led_ind.buffer = (uint8_t *)led7seg_buf;
	led_ind.driver = (LED7SEG_Interface*) &led_driver;


	for (uint8_t i = 0; i < led_ind.num_digits; i++)  led_driver.digit[i] = pins[i];


//	led_driver.interface = LED7SEG_PIN_INTERFACE;

	led_driver.interface = LED7SEG_SR_INTERFACE;

	/*
	led_driver.seg_port = PORT(LED_IND_SEG_A);
	led_driver.seg_masks[LED7SEG_SEG_A] = PIN(LED_IND_SEG_A);
	led_driver.seg_masks[LED7SEG_SEG_B] = PIN(LED_IND_SEG_B);
	led_driver.seg_masks[LED7SEG_SEG_C] = PIN(LED_IND_SEG_C);
	led_driver.seg_masks[LED7SEG_SEG_D] = PIN(LED_IND_SEG_D);
	led_driver.seg_masks[LED7SEG_SEG_E] = PIN(LED_IND_SEG_E);
	led_driver.seg_masks[LED7SEG_SEG_F] = PIN(LED_IND_SEG_F);
	led_driver.seg_masks[LED7SEG_SEG_G] = PIN(LED_IND_SEG_G);
	led_driver.seg_masks[LED7SEG_SEG_H] = PIN(LED_IND_SEG_H);
*/
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

	init_data.usart = USART1;
	init_data.baudrate= 9600;
	init_data.parity = USART_PARITY_NONE;
	init_data.usart_tx = (GPIO_HW_PIN) {USART1_TX_PIN};
	init_data.usart_rx = (GPIO_HW_PIN) {USART1_RX_PIN};
	init_data.rx_buf_ptr = (uint8_t *)usart1_rx_buf;
	init_data.tx_buf_ptr = (uint8_t *)usart1_tx_buf;
	init_data.rx_buf_size = USART1_RX_BUF_SIZE;
	init_data.tx_buf_size = USART1_TX_BUF_SIZE;

	rcc_periph_clock_enable(RCC_USART1);
	uart_init(&usart_1, &init_data);

	usart_disable_rx_interrupt(usart_1.usart );
	usart_hl_set_irq_handler(&usart_1, MB_USART_ISR_FN, &modbus_pr);


	mb_serial_drv.usart_hl = &usart_1;

	nvic_enable_irq(NVIC_USART1_IRQ);


	mb_serial_drv.interface = MODBUS_INTERFACE;
	modbus_pr.driver = (Modbus_Interface*)&mb_serial_drv;
	modbus_pr.packet_buf = mb_packet_buf;

	/* Select either ASCII or RTU Mode. */
	eMBInit(&modbus_pr, 11 );

	/* Initialize the holding register values before starting the
	 * Modbus stack
	 */
	for (uint16_t i = 0; i < REG_HOLDING_NREGS; i++)
	{
		usRegHoldingBuf[i] = (unsigned short) i;
	}
	/* Initialize the input register values before starting the
	 * Modbus stack
	 */
	for (uint16_t i = 0; i < REG_INPUT_NREGS; i++)
	{
		usRegInputBuf[i] = (unsigned short) i;
	}

	/* Enable the Modbus Protocol Stack. */
	eMBEnable(&modbus_pr);

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

/******************************************************************************
void usart_setup(void)
{
	USART_Init		init_data;

	init_data.usart = USART1;
	init_data.baudrate= 9600;
	init_data.parity = USART_PARITY_NONE;
	init_data.usart_tx = (GPIO_HW_PIN) {USART1_TX_PIN};
	init_data.usart_rx = (GPIO_HW_PIN) {USART1_RX_PIN};
	init_data.rx_buf_ptr = (uint8_t *)usart1_rx_buf;
	init_data.tx_buf_ptr = (uint8_t *)usart1_tx_buf;
	init_data.rx_buf_size = USART1_RX_BUF_SIZE;
	init_data.tx_buf_size = USART1_TX_BUF_SIZE;

	rcc_periph_clock_enable(RCC_USART1);
	uart_init(&usart_1, &init_data);
	// Enable the usart1 interrupt.
	usart_hl_set_irq_handler(&usart_1, def_usart_irq_fn, &usart_1);
	nvic_enable_irq(NVIC_USART1_IRQ);
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

//	gpio_mode_setup(PORT(LED_GREEN), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(LED_GREEN));
//	gpio_set(PORT(LED_GREEN), PIN(LED_GREEN));

//	gpio_mode_setup(PORT(LED_GREEN), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(LED_GREEN));
//	gpio_mode_setup(PORT(MOTOR), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(MOTOR));

	// Buttons init
	for (uint8_t i=0; i<4; i++)
		gpio_mode_setup(button_pins[i].port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, button_pins[i].pins);


	delay_timer_init();
	init_owi();
	DS18B20_Init(&t_int_sensor, DS18B20_12BIT_RES);
	DS18B20_Init(&t_ext_sensor, DS18B20_12BIT_RES);

//	rtc_hl_init();
//	rtc_hl_set_time(12,30,0);

//	usart_setup();
//	usart_hl_send_str(&usart_1, (uint8_t *)"MQTT ok");


//	const GPIO_HW_PIN pins[LED7SEG_DIGITS_NUM] = { { LED_IND_DIG_0 }, { LED_IND_DIG_1 }, { LED_IND_DIG_2 } };

	rtc_ds1307.SCL = (GPIO_HW_PIN) {I2C1_SCL};
	rtc_ds1307.SDA = (GPIO_HW_PIN) {I2C1_SDA};

	DS1307_Init(&rtc_ds1307, I2C1, RCC_I2C1, 8);

//	DS1307_Init();
//	xSemaphoreGive(xDS1307_update );

	init_modbus();

	init_led7seg();


}


void vApplicationIdleHook( void )
{
//	gpio_clear(LED_GREEN);
//	GPIO_SET(LED_GREEN);
//	GPIO_CLR(LED_YELLOW);

}

void vApplicationTickHook( void )
{
//	gpio_set(LED_GREEN);
//	GPIO_CLR(LED_GREEN);
//	GPIO_SET(LED_YELLOW);

}

/******************************************************************************/
int main(void)
{
	periphery_init();
//	xTemperQueue=xQueueCreate( 5, sizeof( DS18B20_TypeDef ));
	xTaskCreate(vGetTempTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2 ,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vGetButtonStateTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(vGreenLedTask,(signed char*)"", configMINIMAL_STACK_SIZE,	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(vIndDataOutTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vLed7segUpdateTask,(signed char*)"", configMINIMAL_STACK_SIZE * 1,	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(vModbusTask,(signed char*)"", configMINIMAL_STACK_SIZE * 1,	NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();


	for( ;; );
}
