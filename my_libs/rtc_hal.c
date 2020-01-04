
#include <rtc_hal.h>


/******************************************************************************/
#ifdef STM32F0
void rtc_hl_init()
{
	pwr_disable_backup_domain_write_protect();			// disable backup domain write protection
	rcc_set_rtc_clock_source(RTC_CLOCK_SOURCE);		// select HSE as RTC clock source
	rcc_periph_clock_enable(RCC_RTC);					// enable RTC clock
	rtc_wait_for_synchro();								// wait for RTC registers synchronization
	rtc_unlock();											// Disable the write protection for RTC registers
	RTC_ISR |= RTC_ISR_INIT;								// Set the Initialization mode
	while (!(RTC_ISR & RTC_ISR_INITF));					// Wait till RTC is in INIT state
	rtc_set_prescaler(RTC_SYNCH_PRESCALLER , RTC_ASYNCH_PRESCALLER);			// Configure the RTC PRER
	RTC_ISR &= (uint32_t)~RTC_ISR_INIT;					// Exit Initialization mode
//	pwr_enable_backup_domain_write_protect();			// enable backup domain write protection
}


#else
void rtc_hl_init()
{
	uint8_t bcd_hours, bcd_minutes;

	// reset Backup Domain
//	RCC_BDCR |= RCC_BDCR_BDRST;
//	RCC_BDCR &= ~RCC_BDCR_BDRST;

	rcc_periph_clock_enable(RCC_PWR );				// enable RTC clock
	pwr_disable_backup_domain_write_protect();		// disable backup domain write protection

	/* reset rtc */
	RCC_CSR |= RCC_CSR_RTCRST;
	RCC_CSR &= ~RCC_CSR_RTCRST;

//	rtc_wait_for_synchro();							// wait for RTC registers synchronization

	/* We want to use the LSE fitted on the discovery board */
	rcc_osc_on(RCC_LSE );
	rcc_wait_for_osc_ready(RCC_LSE );

	/* Select the LSE as rtc clock */
	rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE );

	/* ?! Stdperiph examples don't turn this on until _afterwards_ which
	 * simply doesn't work.  It must be on at least to be able to
	 * configure it */
	RCC_CSR |= RCC_CSR_RTCEN;

	rtc_unlock();								// Disable the write protection for RTC registers
	RTC_ISR |= RTC_ISR_INIT;					// Set the Initialization mode
	while (!(RTC_ISR & RTC_ISR_INITF));		// Wait till RTC is in INIT state

	rtc_set_prescaler(255, 127 );				// Configure the RTC PRER
	RTC_ISR &= (uint32_t) ~RTC_ISR_INIT;	// Exit Initialization mode

	rtc_lock();				// and write protect again
//    RCC_CSR |= RCC_CSR_RTCEN;				// and finally enable the clock
	rtc_wait_for_synchro();	// wait for RTC registers synchronization
	//   pwr_enable_backup_domain_write_protect();			// enable backup domain write protection
}
#endif



void rtc_hl_set_time(uint8_t hour, uint8_t minutes, uint8_t seconds)
{
	uint8_t bcd_hour, bcd_minutes, bcd_seconds;
	uint32_t tmp=0;

	rtc_unlock();								// Disable the write protection for RTC registers
	RTC_ISR |= RTC_ISR_INIT;					// Set the Initialization mode
	while (!(RTC_ISR & RTC_ISR_INITF));		// Wait till RTC is in INIT state

	bcd_hour = uint32_to_bcd(hour );
	bcd_minutes = uint32_to_bcd(minutes );
	bcd_seconds = uint32_to_bcd(seconds );

	tmp |= (bcd_hour << RTC_TR_HU_SHIFT) | (bcd_minutes << RTC_TR_MNU_SHIFT)  | (bcd_seconds << RTC_TR_SU_SHIFT);
	RTC_TR = tmp;

	RTC_ISR &= (uint32_t) ~RTC_ISR_INIT;			// Exit Initialization mode
	rtc_lock();									// and write protect again
	rtc_wait_for_synchro();	// wait for RTC registers synchronization
}


void rtc_hl_set_date(uint16_t year, uint8_t month, uint8_t mday, uint8_t wday)
{
	uint8_t 	bcd_year, bcd_month, bcd_mday;
	uint32_t tmp=0;

	rtc_unlock();								// Disable the write protection for RTC registers
	RTC_ISR |= RTC_ISR_INIT;					// Set the Initialization mode
	while (!(RTC_ISR & RTC_ISR_INITF));		// Wait till RTC is in INIT state

	bcd_year = uint32_to_bcd(year % 100 );
	bcd_month = uint32_to_bcd(month );
	bcd_mday = uint32_to_bcd(mday );
//	wday = rtc_hl_calc_dayofweek(year, month, day);

	tmp |= (bcd_year << RTC_DR_YU_SHIFT) | (wday << RTC_DR_WDU_SHIFT)| (bcd_month << RTC_DR_MU_SHIFT)  | (bcd_mday << RTC_DR_DU_SHIFT);
	RTC_DR = tmp;

	RTC_ISR &= (uint32_t) ~RTC_ISR_INIT;			// Exit Initialization mode
	rtc_lock();									// and write protect again
	rtc_wait_for_synchro();	// wait for RTC registers synchronization
}


uint8_t rtc_hl_get_time( uint8_t ret_val)
{
	if(ret_val & RTC_HL_HOURS)
	{
		return 10 * ((RTC_TR >> RTC_TR_HT_SHIFT) & RTC_TR_HT_MASK) + ((RTC_TR >> RTC_TR_HU_SHIFT) & RTC_TR_HU_MASK);
	}
	if(ret_val & RTC_HL_MINUTES)
	{
		return 10 * ( (RTC_TR >> RTC_TR_MNT_SHIFT) & RTC_TR_MNT_MASK) + ((RTC_TR >> RTC_TR_MNU_SHIFT) & RTC_TR_MNU_MASK);
	}
	if(ret_val & RTC_HL_SECONDS)
	{
		return 10 * ( (RTC_TR >> RTC_TR_ST_SHIFT) & RTC_TR_ST_MASK) + ((RTC_TR >> RTC_TR_SU_SHIFT) & RTC_TR_SU_MASK);
	}
	return 0;
}

uint8_t rtc_hl_get_bcd_time( uint8_t ret_val)
{
	if(ret_val & RTC_HL_HOURS)
	{
		return ( ((RTC_TR >> RTC_TR_HT_SHIFT) & RTC_TR_HT_MASK) << 4 ) | ((RTC_TR >> RTC_TR_HU_SHIFT) & RTC_TR_HU_MASK);
	}
	if(ret_val & RTC_HL_MINUTES)
	{
		return ( ( (RTC_TR >> RTC_TR_MNT_SHIFT) & RTC_TR_MNT_MASK) <<4 )| ((RTC_TR >> RTC_TR_MNU_SHIFT) & RTC_TR_MNU_MASK);
	}
	if(ret_val & RTC_HL_SECONDS)
	{
		return (  ( (RTC_TR >> RTC_TR_ST_SHIFT) & RTC_TR_ST_MASK) << 4 ) | ((RTC_TR >> RTC_TR_SU_SHIFT) & RTC_TR_SU_MASK);
	}
	return 0;
}

uint8_t rtc_hl_get_date( uint8_t ret_val)
{
	if(ret_val & RTC_HL_YEAR)
	{
		return 10 * ((RTC_DR >> RTC_DR_YT_SHIFT) & RTC_DR_YT_MASK) + ((RTC_DR >> RTC_DR_YU_SHIFT) & RTC_DR_YU_MASK);
	}
	if(ret_val & RTC_HL_MONTH)
	{
		return 10 * ( (RTC_DR >> RTC_DR_MT_SHIFT) & RTC_DR_MT_MASK) + ((RTC_DR >> RTC_DR_MU_SHIFT) & RTC_DR_MU_MASK);
	}
	if(ret_val & RTC_HL_DAY)
	{
		return 10 * ( (RTC_DR >> RTC_DR_DT_SHIFT) & RTC_DR_DT_MASK) + ((RTC_DR >> RTC_DR_DU_SHIFT) & RTC_DR_DU_MASK);
	}
	return 0;
}

uint8_t *rtc_hl_get_time_string(uint8_t* str, uint8_t ret_val)
{
	uint8_t *ptr;

	ptr = str;

	if(ret_val & RTC_HL_HOURS)
	{
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_HT_SHIFT) & RTC_TR_HT_MASK, ptr );
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_HU_SHIFT) & RTC_TR_HU_MASK, ptr );
	}
	if(ret_val & RTC_HL_MINUTES)
	{
		*ptr++ = ':';
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_MNT_SHIFT) & RTC_TR_MNT_MASK, ptr );
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_MNU_SHIFT) & RTC_TR_MNU_MASK, ptr );
	}
	if(ret_val & RTC_HL_SECONDS)
	{
		*ptr++ = ':';
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_ST_SHIFT) & RTC_TR_ST_MASK, ptr );
		ptr = Uint32ToStr((RTC_TR >> RTC_TR_SU_SHIFT) & RTC_TR_SU_MASK, ptr );
	}

	*ptr = 0;
	return  ptr;
}

uint8_t *rtc_hl_get_date_string(uint8_t* str, uint8_t ret_val)
{
	uint8_t *ptr;

	ptr = str;

	if(ret_val & RTC_HL_DAY)
	{
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_DT_SHIFT) & RTC_DR_DT_MASK, ptr );
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_DU_SHIFT) & RTC_DR_DU_MASK, ptr );
	}
	if(ret_val & RTC_HL_MONTH)
	{
		*ptr++ = '.';
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_MT_SHIFT) & RTC_DR_MT_MASK, ptr );
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_MU_SHIFT) & RTC_DR_MU_MASK, ptr );
	}
	if(ret_val & RTC_HL_YEAR)
	{
		*ptr++ = '.';
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_YT_SHIFT) & RTC_DR_YT_MASK, ptr );
		ptr = Uint32ToStr((RTC_DR >> RTC_DR_YU_SHIFT) & RTC_DR_YU_MASK, ptr );
	}

	*ptr = 0;
	return  ptr;
}


/*******************************************************************************
* Function Name  : isDST
* Description    : checks if given time is in Daylight Saving time-span.
* Input          : time-struct, must be fully populated including weekday
* Output         : none
* Return         : false: no DST ("winter"), true: in DST ("summer")
*  DST according to Ukraine standard
*  Based on code from Peter Dannegger found in the microcontroller.net forum.
*******************************************************************************/
static bool rtc_hl_is_dst(uint8_t month, uint8_t mday, uint8_t wday, uint8_t hour)
{
	if (month < 3 || month > 10)	return false;		// month 1, 2, 11, 12  -> Winter

	if (mday + wday >= 32 && ( (wday != 7) || hour >= 2))		// after last Sunday 2:00
	{
		if (month == 10)	return false;	// October -> Winter
	} else													// before last Sunday 2:00
	{
		if (month == 3)	return false;	// March -> Winter
	}

	return true;
}

void rtc_hl_set_dst(datetime *time, uint32_t *seconds)
{
	if (rtc_hl_is_dst(time->mo, time->dd, time->wd, time->hh ))
	{
		*seconds +=  3600;
	}
}


void rtc_hl_utc2datetime(datetime *time, uint32_t seconds)
{
	uint8_t yf=0;
	uint32_t  n=0,d=0,total_d=0,rz=0;
	uint16_t y=0,r=0,yr=0;
	int32_t yd=0;

	n = seconds;
	total_d = seconds/(SECS_PERDAY);
	d=0;
	uint32_t p_year_total_sec=SECS_PERDAY*365;
	uint32_t r_year_total_sec=SECS_PERDAY*366;
	while(n>=p_year_total_sec)
	{
		if((EPOCH+r)%400==0 || ((EPOCH+r)%100!=0 && (EPOCH+r)%4==0))
		{
			n = n -(r_year_total_sec);
			d = d + 366;
		}
		else
		{
			n = n - (p_year_total_sec);
			d = d + 365;
		}
		r+=1;
		y+=1;

	}

	y += EPOCH;

	time->yy = y;

	yd=0;
	yd = total_d - d;

	yf=1;
	while(yd>=28)
	{

		if(yf==1 || yf==3 || yf==5 || yf==7 || yf==8 || yf==10 || yf==12)
		{
			yd -= 31;
			if(yd<0)break;
			rz += 31;
		}

		if (yf==2)
		{
			if (y%400==0 || (y%100!=0 && y%4==0))
			{
				yd -= 29;
				if(yd<0)break;
				rz += 29;
			}
			else
			{
				yd -= 28;
				if(yd<0)break;
				rz += 28;
			}
		}
		if(yf==4 || yf==6 || yf==9 || yf==11 )
		{
			yd -= 30;
			if(yd<0)break;
			rz += 30;
		}
		yf += 1;

	}
	time->mo=yf;
	yr = total_d-d-rz;

	yr += 1;

	time->dd=yr;
	time->wd = (total_d % 7) +1;    /* day 7 sunday */

	//calculation for time
	seconds = seconds%SECS_PERDAY;
	time->hh = seconds/3600;
	time->mm = (seconds%3600)/60;
	time->ss = (seconds%3600)%60;

}


/*
00)UTC-12:00 Baker Island, Howland Island (both uninhabited)
01) UTC-11:00 American Samoa, Samoa
02) UTC-10:00 (Summer)French Polynesia (most), United States (Aleutian Islands, Hawaii)
03) UTC-09:30 Marquesas Islands
04) UTC-09:00 Gambier Islands;(Summer)United States (most of Alaska)
05) UTC-08:00 (Summer)Canada (most of British Columbia), Mexico (Baja California)
06) UTC-08:00 United States (California, most of Nevada, most of Oregon, Washington (state))
07) UTC-07:00 Mexico (Sonora), United States (Arizona); (Summer)Canada (Alberta)
08) UTC-07:00 Mexico (Chihuahua), United States (Colorado)
09) UTC-06:00 Costa Rica, El Salvador, Ecuador (Galapagos Islands), Guatemala, Honduras
10) UTC-06:00 Mexico (most), Nicaragua;(Summer)Canada (Manitoba, Saskatchewan), United States (Illinois, most of Texas)
11) UTC-05:00 Colombia, Cuba, Ecuador (continental), Haiti, Jamaica, Panama, Peru
12) UTC-05:00 (Summer)Canada (most of Ontario, most of Quebec)
13) UTC-05:00 United States (most of Florida, Georgia, Massachusetts, most of Michigan, New York, North Carolina, Ohio, Washington D.C.)
14) UTC-04:30 Venezuela
15) UTC-04:00 Bolivia, Brazil (Amazonas), Chile (continental), Dominican Republic, Canada (Nova Scotia), Paraguay,
16) UTC-04:00 Puerto Rico, Trinidad and Tobago
17) UTC-03:30 Canada (Newfoundland)
18) UTC-03:00 Argentina; (Summer) Brazil (Brasilia, Rio de Janeiro, Sao Paulo), most of Greenland, Uruguay
19) UTC-02:00 Brazil (Fernando de Noronha), South Georgia and the South Sandwich Islands
20) UTC-01:00 Portugal (Azores), Cape Verde
21) UTC&#177;00:00 Cote d'Ivoire, Faroe Islands, Ghana, Iceland, Senegal; (Summer) Ireland, Portugal (continental and Madeira)
22) UTC&#177;00:00 Spain (Canary Islands), Morocco, United Kingdom
23) UTC+01:00 Angola, Cameroon, Nigeria, Tunisia; (Summer)Albania, Algeria, Austria, Belgium, Bosnia and Herzegovina,
24) UTC+01:00 Spain (continental), Croatia, Czech Republic, Denmark, Germany, Hungary, Italy, Kinshasa, Kosovo,
25) UTC+01:00 Macedonia, France (metropolitan), the Netherlands, Norway, Poland, Serbia, Slovakia, Slovenia, Sweden, Switzerland
26) UTC+02:00 Libya, Egypt, Malawi, Mozambique, South Africa, Zambia, Zimbabwe, (Summer)Bulgaria, Cyprus, Estonia,
27) UTC+02:00 Finland, Greece, Israel, Jordan, Latvia, Lebanon, Lithuania, Moldova, Palestine, Romania, Syria, Turkey, Ukraine
28) UTC+03:00 Belarus, Djibouti, Eritrea, Ethiopia, Iraq, Kenya, Madagascar, Russia (Kaliningrad Oblast), Saudi Arabia,
29) UTC+03:00 South Sudan, Sudan, Somalia, South Sudan, Tanzania, Uganda, Yemen
30) UTC+03:30 (Summer)Iran
31) UTC+04:00 Armenia, Azerbaijan, Georgia, Mauritius, Oman, Russia (European), Seychelles, United Arab Emirates
32) UTC+04:30 Afghanistan
33) UTC+05:00 Kazakhstan (West), Maldives, Pakistan, Uzbekistan
34) UTC+05:30 India, Sri Lanka
35) UTC+05:45 Nepal
36) UTC+06:00 Kazakhstan (most), Bangladesh, Russia (Ural: Sverdlovsk Oblast, Chelyabinsk Oblast)
37) UTC+06:30 Cocos Islands, Myanmar
38) UTC+07:00 Jakarta, Russia (Novosibirsk Oblast), Thailand, Vietnam
39) UTC+08:00 China, Hong Kong, Russia (Krasnoyarsk Krai), Malaysia, Philippines, Singapore, Taiwan, most of Mongolia, Western Australia
40) UTC+09:00 Korea, East Timor, Russia (Irkutsk Oblast), Japan
41) UTC+09:30 Australia (Northern Territory);(Summer)Australia (South Australia))
42) UTC+10:00 Russia (Zabaykalsky Krai); (Summer)Australia (New South Wales, Queensland, Tasmania, Victoria)
43) UTC+10:30 Lord Howe Island
44) UTC+11:00 New Caledonia, Russia (Primorsky Krai), Solomon Islands
45) UTC+11:30 Norfolk Island
46) UTC+12:00 Fiji, Russia (Kamchatka Krai);(Summer)New Zealand
47) UTC+12:45 (Summer)New Zealand
48) UTC+13:00 Tonga
49) UTC+14:00 Kiribati (Line Islands)
*/
void rtc_hl_add_timezone(uint8_t time_zone, uint32_t *seconds)
{

	switch (time_zone)
	{
	case 0:
		*seconds -=  12*3600;
		break;
	case 1:
		*seconds -=  11*3600;
		break;
	case 2:
		*seconds -=  10*3600;
		break;
	case 3:
		*seconds -=  (9*3600+30*60);
		break;
	case 4:
		*seconds -=  9*3600;
		break;
	case 5:
	case 6:
		*seconds -=  8*3600;
		break;
	case 7:
	case 8:
		*seconds -=  7*3600;
		break;
	case 9:
	case 10:
		*seconds -=  6*3600;
		break;
	case 11:
	case 12:
	case 13:
		*seconds -= 5*3600;
		break;
	case 14:
		*seconds -=  (4*3600+30*60);
		break;
	case 15:
	case 16:
		*seconds -=  4*3600;
		break;
	case 17:
		*seconds -=  (3*3600+30*60);
		break;
	case 18:
		*seconds -=  3*3600;
		break;
	case 19:
		*seconds -=  2*3600;
		break;
	case 20:
		*seconds -=  1*3600;
		break;
	case 21:
	case 22:
		break;
	case 23:
	case 24:
	case 25:
		*seconds +=  1*3600;
		break;
	case 26:
	case 27:
		*seconds +=  2*3600;
		break;
	case 28:
	case 29:
		*seconds +=  3*3600;
		break;
	case 30:
		*seconds +=  (3*3600+30*60);
		break;
	case 31:
		*seconds +=  4*3600;
		break;
	case 32:
		*seconds +=  (4*3600+30*60);
		break;
	case 33:
		*seconds +=  5*3600;
		break;
	case 34:
		*seconds +=  (5*3600+30*60);
		break;
	case 35:
		*seconds +=  (5*3600+45*60);
		break;
	case 36:
		*seconds +=  6*3600;
		break;
	case 37:
		*seconds +=  (6*3600+30*60);
		break;
	case 38:
		*seconds +=  7*3600;
		break;
	case 39:
		*seconds +=  8*3600;
		break;
	case 40:
		*seconds +=  9*3600;
		break;
	case 41:
		*seconds +=  (9*3600+30*60);
		break;
	case 42:
		*seconds +=  10*3600;
		break;
	case 43:
		*seconds +=  (10*3600+30*60);
		break;
	case 44:
		*seconds +=  11*3600;
		break;
	case 45:
		*seconds +=  (11*3600+30*60);
		break;
	case 46:
		*seconds +=  12*3600;
		break;
	case 47:
		*seconds +=  (12*3600+45*60);
		break;
	case 48:
		*seconds +=  13*3600;
		break;
	case 49:
		*seconds +=  14*3600;
		break;
	}

}


/*
tstamp changedatetime_to_seconds(void)
{
	tstamp seconds=0;
	uint32_t total_day=0;
	uint16_t i=0,run_year_cnt=0,l=0;

	l = Nowdatetime.yy;//low


	for(i=EPOCH;i<l;i++)
	{
		if((i%400==0) || ((i%100!=0) && (i%4==0)))
		{
			run_year_cnt += 1;
		}
	}

	total_day=(l-EPOCH-run_year_cnt)*365+run_year_cnt*366;

	for(i=1;i<=Nowdatetime.mo;i++)
	{
		if(i==5 || i==7 || i==10 || i==12)
		{
			total_day += 30;
		}
		if (i==3)
		{
			if (l%400==0 && l%100!=0 && l%4==0)
			{
				total_day += 29;
			}
			else
			{
				total_day += 28;
			}
		}
		if(i==2 || i==4 || i==6 || i==8 || i==9 || i==11)
		{
			total_day += 31;
		}
	}

	seconds = (total_day+Nowdatetime.dd-1)*24*3600;
	seconds += Nowdatetime.ss;//seconds
	seconds += Nowdatetime.mm*60;//minute
	seconds += Nowdatetime.hh*3600;//hour

	return seconds;
}
*/

