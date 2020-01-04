//****************************************************************************
//
//! \file w5500_spi.c
//! \brief WIZnet W5500 EVB spi interface file
//!
//! Copyright (c)  2014, WIZnet Co., LTD.
//! All rights reserved.
//
//****************************************************************************

#include "../my_libs/w5500_spi.h"


/**
  * @brief  Initializes the peripherals used by the W5500 driver.
  * @param  None
  * @retval None
  */


void  wizchip_select(void)
{
	gpio_clear(W5500_SPI_CS);
}

void  wizchip_deselect(void)
{
	gpio_set(W5500_SPI_CS);
}

uint8_t wizchip_read(void)
{
	return spi_xfer(W5500_SPI, 0x00);
}

void  wizchip_write(uint8_t data)
{
	spi_xfer(W5500_SPI,  data);
}

void wizchip_spi_init(void)
{

	rcc_periph_clock_enable(W5500_SPI_RCC);

	gpio_mode_setup(PORT(W5500_SPI_SCK), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(W5500_SPI_SCK)); // SCK
	gpio_set_output_options(PORT(W5500_SPI_SCK), GPIO_OTYPE_PP, GPIO_OSPEED_40MHZ, PIN(W5500_SPI_SCK));
	gpio_set_af(PORT(W5500_SPI_SCK ), W5500_AF, PIN(W5500_SPI_SCK ) );

	gpio_mode_setup(PORT(W5500_SPI_MISO), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(W5500_SPI_MISO));	// MISO
	gpio_set_output_options(PORT(W5500_SPI_MISO), GPIO_OTYPE_PP, GPIO_OSPEED_40MHZ, PIN(W5500_SPI_MISO));
	gpio_set_af(PORT(W5500_SPI_MISO ), W5500_AF, PIN(W5500_SPI_MISO ) );

	gpio_mode_setup(PORT(W5500_SPI_MOSI), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(W5500_SPI_MOSI));	// MOSI
	gpio_set_output_options(PORT(W5500_SPI_MOSI), GPIO_OTYPE_PP, GPIO_OSPEED_40MHZ, PIN(W5500_SPI_MOSI));
	gpio_set_af(PORT(W5500_SPI_MOSI ), W5500_AF, PIN(W5500_SPI_MOSI ) );

	gpio_mode_setup(PORT(W5500_SPI_CS), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(W5500_SPI_CS));	// CS
	gpio_set_output_options(PORT(W5500_SPI_CS), GPIO_OTYPE_PP, GPIO_OSPEED_40MHZ, PIN(W5500_SPI_CS));
	wizchip_deselect();
	gpio_mode_setup(PORT(W5500_RESET), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(W5500_RESET));
	gpio_set(W5500_RESET);


	spi_reset(W5500_SPI);
	spi_set_master_mode(W5500_SPI);
//	spi_set_unidirectional_mode(W5500_SPI);
//	spi_set_full_duplex_mode(W5500_SPI);
#ifdef STM32F0
	spi_set_data_size(W5500_SPI,SPI_CR2_DS_8BIT);
#else
	spi_set_dff_8bit(W5500_SPI);
#endif
//	spi_fifo_reception_threshold_8bit(W5500_SPI);
	spi_set_clock_polarity_0(W5500_SPI);
	spi_set_clock_phase_0(W5500_SPI);
	spi_enable_software_slave_management(W5500_SPI);
	spi_set_nss_high(W5500_SPI);
	spi_set_baudrate_prescaler(W5500_SPI,SPI_CR1_BR_FPCLK_DIV_4);
	spi_send_msb_first(W5500_SPI);
//	spi_enable_crc(W5500_SPI);
	spi_enable(W5500_SPI);

}

void w5500_init(void (*delay_ms)(uint16_t) )
{
	uint8_t memsize[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };

	wizchip_spi_init();

	gpio_clear(W5500_RESET);	// N_RESET
	delay_ms(50);
	gpio_set(W5500_RESET);	// N_RESET
	delay_ms(300);

	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

	/* wizchip initialize*/
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1) {
//		printf("WIZCHIP Initialized fail.\r\n");
		while (1);
	}
/*
	do {
		if (ctlwizchip(CW_GET_PHYLINK, (void*) &tmp) == -1)
			printf("Unknown PHY Link stauts.\r\n");
	} while (tmp == PHY_LINK_OFF);
*/
}
