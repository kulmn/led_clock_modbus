/*
 * hc595.c
 *
 *  Created on: 3 ����. 2016 �.
 *      Author: kulish_y
 */

#include "../my_libs/hc595.h"

/******************************************************************************/
void HC595_spi_init(void)
{
	// Enable SPI1 Periph and gpio clocks
	rcc_periph_clock_enable(HC595_SPI_RCC);

	gpio_mode_setup(PORT(HC595_SPI_SCK), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(HC595_SPI_SCK)); // SCK
	gpio_set_output_options(PORT(HC595_SPI_SCK), GPIO_OTYPE_PP, HC595_GPIO_SPEED, PIN(HC595_SPI_SCK));
	gpio_set_af(PORT(HC595_SPI_SCK ), HC595_AF, PIN(HC595_SPI_SCK ) );

	gpio_mode_setup(PORT(HC595_SPI_MOSI), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(HC595_SPI_MOSI));	// MOSI
	gpio_set_output_options(PORT(HC595_SPI_MOSI), GPIO_OTYPE_PP, HC595_GPIO_SPEED, PIN(HC595_SPI_MOSI));
	gpio_set_af(PORT(HC595_SPI_MOSI ), HC595_AF, PIN(HC595_SPI_MOSI ) );

	gpio_mode_setup(PORT(HC595_SPI_CS), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(HC595_SPI_CS));	// CS
	gpio_set_output_options(PORT(HC595_SPI_CS), GPIO_OTYPE_PP, HC595_GPIO_SPEED, PIN(HC595_SPI_CS));

	gpio_set(HC595_SPI_CS);

//	spi_reset(SPI1);
	spi_set_master_mode(HC595_SPI);

	spi_set_bidirectional_transmit_only_mode(HC595_SPI);

//	spi_set_unidirectional_mode(SPI1);
//	spi_set_receive_only_mode(SPI1);
#ifdef STM32F0
	spi_set_data_size(HC595_SPI,SPI_CR2_DS_16BIT);
#else
	spi_set_dff_16bit(HC595_SPI);
#endif
	spi_set_clock_polarity_0(HC595_SPI);
	spi_set_clock_phase_0(HC595_SPI);
	spi_enable_software_slave_management(HC595_SPI);
	spi_set_nss_high(HC595_SPI);
	spi_set_baudrate_prescaler(HC595_SPI,SPI_CR1_BR_FPCLK_DIV_8);
	spi_send_msb_first(HC595_SPI);
	spi_disable_crc(HC595_SPI);

	spi_enable(HC595_SPI);

}

/******************************************************************************/
void HC595_send(uint16_t data)
{
	gpio_clear(HC595_SPI_CS);
	spi_send(HC595_SPI,data);
//	spi_send8(SPI1,data);
	while ((SPI_SR(HC595_SPI) & SPI_SR_BSY));
	gpio_set(HC595_SPI_CS);
}

/******************************************************************************
void HC595_send(uint16_t data)
{
	gpio_clear(HC595_SPI_CS);
	spi_send(SPI1,data);
//	spi_send8(SPI1,data);
	while ((SPI_SR(SPI1) & SPI_SR_BSY));
	gpio_set(HC595_SPI_CS);
}
*/

