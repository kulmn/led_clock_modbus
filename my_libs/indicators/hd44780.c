#include <stdlib.h>
#include <stdint.h>
#include "hd44780.h"



HD44780_Result hd44780_send_cmd(HD44780 *display, uint8_t value);
HD44780_Result hd44780_send_data(HD44780 *display, uint8_t value);

HD44780_Result hd44780_init(HD44780 *display, HD44780_Mode addr_mode, uint8_t columns, uint8_t rows, HD44780_CharSize charsize)
{
	if (addr_mode == HD44780_MODE_8BIT) display->displayfunction |= HD44780_FLAG_8BITMODE;

	/* For some 1 line displays you can select a 10 pixel high font */
	if (charsize == HD44780_CHARSIZE_5x10 && rows == 1) display->displayfunction |= HD44780_FLAG_5x10DOTS;

	if (rows > 1) display->displayfunction |= HD44780_FLAG_2LINE;

	/* SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	 * according to datasheet, we need at least 40ms after power rises above 2.7V
	 * before sending commands.
	 */
	display->delay_microseconds(10000 );
	display->delay_microseconds(10000 );
	display->delay_microseconds(10000 );
	display->delay_microseconds(10000 );
	display->delay_microseconds(10000 );

	display->driver->hd44780_driver_init(display->driver, addr_mode);

	/* Finally, set # lines, font size, etc. */
	HD44780_RETURN_IF_ERROR(hd44780_send_cmd(display, HD44780_CMD_FUNCTIONSET | display->displayfunction) );

	/* Turn the display on with no cursor or blinking default */
	display->displaycontrol = HD44780_FLAG_DISPLAYON | HD44780_FLAG_CURSOROFF | HD44780_FLAG_BLINKOFF;
	HD44780_RETURN_IF_ERROR(hd44780_display_on(display ) );

	/* Clear it off */
	HD44780_RETURN_IF_ERROR(hd44780_clear(display ) );
	/* Initialize to default text direction (for romance languages) */
	display->displaymode = HD44780_FLAG_ENTRYLEFT | HD44780_FLAG_ENTRYSHIFTDECREMENT;
	/* Set the entry mode */
	HD44780_RETURN_IF_ERROR(hd44780_send_cmd(display, HD44780_CMD_ENTRYMODESET | display->displaymode) );

	return HD44780_RESULT_OK;
}

HD44780_Result hd44780_write_char(HD44780 *display, char c)
{
	return hd44780_send_data(display, (uint8_t) c );
}

HD44780_Result hd44780_write_string(HD44780 *display, const char *s)
{
	while (*s)
		HD44780_RETURN_IF_ERROR(hd44780_write_char(display, *s++ ) );

	return HD44780_RESULT_OK;
}

HD44780_Result hd44780_clear(HD44780 *display)
{
	HD44780_RETURN_IF_ERROR(hd44780_send_cmd(display, HD44780_CMD_CLEARDISPLAY) ); // clear display, set cursor position to zero
	display->delay_microseconds(3000 ); // this command takes a long time!

	return HD44780_RESULT_OK;
}

HD44780_Result hd44780_home(HD44780 *display)
{
	HD44780_RETURN_IF_ERROR(hd44780_send_cmd(display, HD44780_CMD_RETURNHOME) );
	display->delay_microseconds(3000 );  // this command takes a long time!

	return HD44780_RESULT_OK;
}

HD44780_Result hd44780_scroll_left(HD44780 *display)
{
	return hd44780_send_cmd(display,
	HD44780_CMD_CURSORSHIFT | HD44780_FLAG_DISPLAYMOVE | HD44780_FLAG_MOVELEFT );
}

HD44780_Result hd44780_scroll_right(HD44780 *display)
{
	return hd44780_send_cmd(display,
	HD44780_CMD_CURSORSHIFT | HD44780_FLAG_DISPLAYMOVE | HD44780_FLAG_MOVERIGHT );
}

HD44780_Result hd44780_left_to_right(HD44780 *display)
{
	display->displaymode |= HD44780_FLAG_ENTRYLEFT;
	return hd44780_send_cmd(display, HD44780_CMD_ENTRYMODESET | display->displaymode );
}

HD44780_Result hd44780_right_to_left(HD44780 *display)
{

	display->displaymode &= ~HD44780_FLAG_ENTRYLEFT;
	return hd44780_send_cmd(display, HD44780_CMD_ENTRYMODESET | display->displaymode );
}

/* FIXME moves the cursor out of screen */
HD44780_Result hd44780_create_char(HD44780 *display, uint8_t location, const uint8_t *charmap)
{
	location &= 0x7; // we only have 8 locations 0-7
	HD44780_RETURN_IF_ERROR(hd44780_send_cmd(display, HD44780_CMD_SETCGRAMADDR | (location << 3)) );

	for (unsigned i = 0; i < 8; ++i)
		HD44780_RETURN_IF_ERROR(hd44780_send_data(display, charmap[i] ) );

	return HD44780_RESULT_OK;
}

HD44780_Result hd44780_move_cursor(HD44780 *display, uint8_t column, uint8_t row)
{
	static const int row_offsets[] = { 0x00, 0x40, 0x10, 0x50 };

	return hd44780_send_cmd(display, HD44780_CMD_SETDDRAMADDR | (column + row_offsets[row]) );
}

HD44780_Result hd44780_display_on(HD44780 *display)
{
	display->displaycontrol |= HD44780_FLAG_DISPLAYON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_display_off(HD44780 *display)
{
	display->displaycontrol &= ~HD44780_FLAG_DISPLAYON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_blink_on(HD44780 *display)
{
	display->displaycontrol |= HD44780_FLAG_BLINKON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_blink_off(HD44780 *display)
{
	display->displaycontrol &= ~HD44780_FLAG_BLINKON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_cursor_on(HD44780 *display)
{
	display->displaycontrol |= HD44780_FLAG_CURSORON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_cursor_off(HD44780 *display)
{
	display->displaycontrol &= ~HD44780_FLAG_CURSORON;
	return hd44780_send_cmd(display, HD44780_CMD_DISPLAYCONTROL | display->displaycontrol );
}

HD44780_Result hd44780_autoscroll_on(HD44780 *display)
{
	display->displaymode |= HD44780_FLAG_ENTRYSHIFTINCREMENT;
	return hd44780_send_cmd(display, HD44780_CMD_ENTRYMODESET | display->displaymode );
}

HD44780_Result hd44780_autoscroll_off(HD44780 *display)
{
	display->displaymode &= ~HD44780_FLAG_ENTRYSHIFTINCREMENT;
	return hd44780_send_cmd(display, HD44780_CMD_ENTRYMODESET | display->displaymode );
}

/*----------------------*/
/*     Internal API     */
/*----------------------*/

HD44780_Result hd44780_send_cmd(HD44780 *display, uint8_t value)
{
	return display->driver->hd44780_driver_send(display->driver, value, HD44780_PINSTATE_LOW );
}

HD44780_Result hd44780_send_data(HD44780 *display, uint8_t value)
{
	return display->driver->hd44780_driver_send(display->driver, value, HD44780_PINSTATE_HIGH );
}

