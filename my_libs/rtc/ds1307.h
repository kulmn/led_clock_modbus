/**
  * @brief   DS1307 Library for STM32xx
 *	
 */
#ifndef DS1307_H
#define DS1307_H


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "convert_fn.h"

#ifdef USE_FREE_RTOS		// FreeRTOS include
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif


//#define DS1307_I2C					I2C1

// DS1307 I2C clock
//#define DS1307_I2C_CLOCK			100000


//#define DS1307_I2C_SCL			GPIOA, GPIO9
//#define DS1307_I2C_SDA			GPIOA, GPIO10
//#define DS1307_I2C_AF			GPIO_AF4



/* I2C slave address for DS1307 */
#define DS1307_I2C_ADDR				0x68

/* Registers location
#define DS1307_SECONDS				0x00
#define DS1307_MINUTES				0x01
#define DS1307_HOURS				0x02
#define DS1307_DAY					0x03
#define DS1307_DATE					0x04
#define DS1307_MONTH				0x05
#define DS1307_YEAR					0x06
#define DS1307_CONTROL				0x07
*/

/* Bits in control register */
#define DS1307_CONTROL_OUT			7
#define DS1307_CONTROL_SQWE			4
#define DS1307_CONTROL_RS1			1
#define DS1307_CONTROL_RS0			0


// Registers location
typedef enum {
	DS1307_SECONDS = 0x00,
	DS1307_MINUTES,
	DS1307_HOURS,
	DS1307_WEEK_DAY,
	DS1307_MOUNTH_DAY,
	DS1307_MONTH,
	DS1307_YEAR,
	DS1307_CONTROL
} DS1307_Reg;


/**
 * @brief  Result enumeration
 */
typedef enum {
	TM_DS1307_Result_Ok = 0x00,         /*!< Everything OK */
	TM_DS1307_Result_Error,             /*!< An error occurred */
	TM_DS1307_Result_DeviceNotConnected /*!< Device is not connected */
} TM_DS1307_Result_t;

/**
 * @brief  Structure for date/time
 */
typedef struct {
	uint8_t seconds; /*!< Seconds parameter, from 00 to 59 */
	uint8_t minutes; /*!< Minutes parameter, from 00 to 59 */
	uint8_t hours;   /*!< Hours parameter, 24Hour mode, 00 to 23 */
	uint8_t week_day;     /*!< Day in a week, from 1 to 7 */
	uint8_t month_day;    /*!< Date in a month, 1 to 31 */
	uint8_t month;   /*!< Month in a year, 1 to 12 */
	uint8_t year;    /*!< Year parameter, 00 to 99, 00 is 2000 and 99 is 2099 */
	uint8_t control;    // control register
} DS1307_DATA;



typedef struct {
	uint32_t					I2C;
	GPIO_HW_PIN			SCL;
	GPIO_HW_PIN			SDA;
//	uint8_t					address;
//	DS1307_DATA			data;
	SemaphoreHandle_t 		xRTC_Mutex;

} DS1307;


/**
 * @defgroup TM_DS1307_Functions
 * @brief    Library Functions
 * @{
 */
 
void DS1307_Init(DS1307 *rtc, uint32_t i2c, enum rcc_periph_clken rcc_i2c, uint32_t clock_megahz);
void DS1307_Get_All_Registers(DS1307 *rtc, DS1307_DATA *data);
void DS1307_Set_All_Registers(DS1307 *rtc, DS1307_DATA *data);
void DS1307_SetReg(DS1307 *rtc, DS1307_Reg reg, uint8_t data, uint8_t bcd_conv);
uint8_t DS1307_GetReg(DS1307 *rtc, DS1307_Reg reg, uint8_t bcd_conv);


/**
 * @brief  Checks for min and max values, which can be stored to DS1307
 * @param  val: Value to be checked
 * @param  min: Minimal allowed value of val
 * @param  max: Maximal allowed value of val
 * @retval Value between min and max
 */
uint8_t TM_DS1307_CheckMinMax(uint8_t val, uint8_t min, uint8_t max);



#endif

