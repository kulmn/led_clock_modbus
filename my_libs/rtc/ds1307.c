
#include <rtc/ds1307.h>


void DS1307_Init(DS1307 *rtc, uint32_t i2c, enum rcc_periph_clken rcc_i2c, uint32_t clock_megahz)
{
	rtc->I2C = i2c;

	rcc_periph_clock_enable(rcc_i2c );
	rcc_set_i2c_clock_hsi(rtc->I2C );

	i2c_reset(rtc->I2C );
	// Setup GPIO
	gpio_mode_setup(rtc->SCL.port, GPIO_MODE_AF, GPIO_PUPD_NONE, rtc->SCL.pins);
	gpio_set_output_options(rtc->SCL.port, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, rtc->SCL.pins);
	gpio_set_af(rtc->SCL.port, rtc->SCL.alt_func, rtc->SCL.pins );
	gpio_mode_setup(rtc->SDA.port, GPIO_MODE_AF, GPIO_PUPD_NONE, rtc->SDA.pins);
	gpio_set_output_options(rtc->SDA.port, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, rtc->SDA.pins);
	gpio_set_af(rtc->SDA.port, rtc->SDA.alt_func, rtc->SDA.pins );

	i2c_peripheral_disable(rtc->I2C );
	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(rtc->I2C );
	i2c_set_digital_filter(rtc->I2C, 0 );
	/* HSI is at 8Mhz */
	i2c_set_speed(rtc->I2C, i2c_speed_sm_100k, clock_megahz );
	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(rtc->I2C );
	//addressing mode
	i2c_set_7bit_addr_mode(rtc->I2C );
	i2c_peripheral_enable(rtc->I2C );

	rtc->xRTC_Mutex = xSemaphoreCreateMutex( );
}

void DS1307_Get_All_Registers(DS1307 *rtc, DS1307_DATA *data)
{
	uint8_t reg = 0;

	if (xSemaphoreTake(rtc->xRTC_Mutex, (TickType_t) 10 ) == pdTRUE)
	{
		// We were able to obtain the semaphore and can now access the shared resource.
		i2c_transfer7(rtc->I2C, DS1307_I2C_ADDR, &reg, 1, (uint8_t*) data, 8 );
		// We have finished accessing the shared resource.  Release the  semaphore. */
		xSemaphoreGive(rtc->xRTC_Mutex );
	} else
	{
		// We could not obtain the semaphore and can therefore not access  the shared resource safely.
	}
//	}
}

void DS1307_Set_All_Registers(DS1307 *rtc, DS1307_DATA *data)
{
	uint8_t *data_buf = (uint8_t*) data;
	uint8_t tx_buf[9], reg = 0;

	// We were able to obtain the semaphore and can now access the shared resource.
	if (xSemaphoreTake(rtc->xRTC_Mutex, (TickType_t) 10 ) == pdTRUE)
	{
		tx_buf[0] = 0;
		for (uint8_t i = 1; i < 9; i++) 	tx_buf[i] = *data_buf++;

		i2c_transfer7(rtc->I2C, DS1307_I2C_ADDR, tx_buf, 9, 0, 0 );
		// We have finished accessing the shared resource.  Release the  semaphore. */
		xSemaphoreGive(rtc->xRTC_Mutex );
	} else
	{
		// We could not obtain the semaphore and can therefore not access  the shared resource safely.
	}
}


/******************************************************************************/
void DS1307_SetReg(DS1307 *rtc, DS1307_Reg reg, uint8_t data, uint8_t bcd_conv)
{
	uint8_t tx_buf[2];
	if (xSemaphoreTake(rtc->xRTC_Mutex, (TickType_t) 10 ) == pdTRUE)
	{
		tx_buf[0] = reg;
		if (bcd_conv) tx_buf[1] = uint32_to_bcd(data );
		else tx_buf[1] = data;

		i2c_transfer7(rtc->I2C, DS1307_I2C_ADDR, tx_buf, 2, 0, 0 );
		xSemaphoreGive(rtc->xRTC_Mutex );
	} else
	{
		// We could not obtain the semaphore and can therefore not access  the shared resource safely.
	}
}



/******************************************************************************/
uint8_t DS1307_GetReg(DS1307 *rtc, DS1307_Reg reg, uint8_t bcd_conv)
{
	uint8_t rx_buf;
	if (xSemaphoreTake(rtc->xRTC_Mutex, (TickType_t) 10 ) == pdTRUE)
	{
		i2c_transfer7(rtc->I2C, DS1307_I2C_ADDR, &reg, 1, &rx_buf, 1 );
		if (bcd_conv) rx_buf = bcd_to_uint8(rx_buf );
		xSemaphoreGive(rtc->xRTC_Mutex );
	} else
	{
		// We could not obtain the semaphore and can therefore not access  the shared resource safely.
	}

	return rx_buf;
}


