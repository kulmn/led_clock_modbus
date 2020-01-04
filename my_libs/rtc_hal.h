#ifndef __RTC_HAL_H_
#define __RTC_HAL_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>

#include <convert_fn.h>


#define RTC_CLOCK_SOURCE					RCC_HSE		// RCC_HSE (/32), RCC_HSI, RCC_LSI
#define RTC_ASYNCH_PRESCALLER			(125-1)			// 7 bit
#define RTC_SYNCH_PRESCALLER				(2000-1)			// 15 bit




#define RTC_HL_HOURS			(0x01)
#define RTC_HL_MINUTES		(0x02)
#define RTC_HL_SECONDS		(0x04)
#define RTC_HL_HMS			(0x07)

#define RTC_HL_YEAR			(0x01)
#define RTC_HL_MONTH			(0x02)
#define RTC_HL_DAY				(0x04)
#define RTC_HL_DMY			(0x07)

#define SECS_PERDAY		86400UL             	// seconds in a day = 60*60*24
#define EPOCH			1900                    // NTP start year

typedef struct _datetime
{
	uint16_t yy;
	uint8_t mo;
	uint8_t dd;
	uint8_t wd;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
} datetime;


void rtc_hl_init();
void rtc_hl_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void rtc_hl_set_date(uint16_t year, uint8_t month, uint8_t day, uint8_t wday);
uint8_t rtc_hl_get_time( uint8_t ret_val);
uint8_t rtc_hl_get_bcd_time( uint8_t ret_val);
uint8_t rtc_hl_get_date( uint8_t ret_val);
uint8_t *rtc_hl_get_time_string(uint8_t* str, uint8_t ret_val);
uint8_t *rtc_hl_get_date_string(uint8_t* str, uint8_t ret_val);
void rtc_hl_set_dst(datetime *time, uint32_t *seconds);
void rtc_hl_utc2datetime(datetime *time, uint32_t seconds);
void rtc_hl_add_timezone(uint8_t time_zone, uint32_t *seconds);

//tstamp changedatetime_to_seconds(void);

#endif
