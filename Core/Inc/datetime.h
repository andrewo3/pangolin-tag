#ifndef DATETIME_H
#define DATETIME_H

#include "stm32l4xx_hal_rtc.h"

const int CENTURY = 20;
const int EPOCH_YEAR = 2000;
const int SECONDS_PER_YEAR = 31536000;
const int SECONDS_PER_DAY = 86400;
const int SECONDS_PER_HOUR = 3600;
const int SECONDS_PER_MINUTE = 60;
const int MONTH_DAYS[12] = {31,28,31,30,31,30,31,31,30,31,30,31};


typedef struct {
	RTC_TimeTypeDef ti;
	RTC_DateTypeDef da;
} DateTime;

long toepoch(DateTime tida) {
	RTC_TimeTypeDef ti = tida.ti;
	RTC_DateTypeDef da = tida.da;
	int current_year = CENTURY * 100 + da.Year;
	int num_years = current_year - EPOCH_YEAR;
	int num_leap_years = 1+ (num_years) / 4; // add 1 because 2000 was a leap year

	//add up seconds from all years (no leap days)
	long second_count = num_years * SECONDS_PER_YEAR;

	//count leap days
	second_count += SECONDS_PER_DAY * num_leap_years;

	//if year is a leap year, remove extra count unless after Feb 29
	if (current_year % 4 == 0 && da.Month < 3) {
		second_count -= SECONDS_PER_DAY;
	}

	//count months so far this year
	for (int m = 0; m < da.Month - 1; m++) {
		second_count += MONTH_DAYS[m] * SECONDS_PER_DAY;
	}

	//count days so far in current month
	second_count += SECONDS_PER_DAY * (da.Date - 1);

	//count hours so far into current day
	second_count += SECONDS_PER_HOUR * ((ti.Hours % 12) + 12 * ti.TimeFormat);

	//count minutes so far into current hour
	second_count += SECONDS_PER_MINUTE * ti.Minutes;

	//count seconds into current minute
	second_count += ti.Seconds;

	return second_count;

}


DateTime fromepoch(long ep) {
	DateTime tida = {0};
	long leap_cycle = 4 * SECONDS_PER_YEAR + SECONDS_PER_DAY;
	long num_leap_cycles = ep / leap_cycle;

	// calculate minimum possible year from number of 4 year cycles
	int year = EPOCH_YEAR + 4 * num_leap_cycles;

	//find remaining number of years after last leap year
	ep %= leap_cycle;

	int y = 0;
	while (ep >= 0 && y < 4) {
		ep -= SECONDS_PER_YEAR;
		if (y == 0) {
			ep -= SECONDS_PER_DAY;
		}
		y++;
	}
	ep += SECONDS_PER_YEAR;
	if (y == 1) {
		ep += SECONDS_PER_DAY;
	}
	year += y - 1;
	tida.da.Year = year % 100;

	//find current month
	int m = 0;
	int month;
	for (month = 0; m <= ep && month < 12; month++) {
		m += MONTH_DAYS[month] * SECONDS_PER_DAY;
		//add extra day for february in a leap year
		if (year % 4 == 0 && month == 1) {
			m += SECONDS_PER_DAY;
		}
	}

	//remove overcount
	m -= MONTH_DAYS[month - 1] * SECONDS_PER_DAY;
	// remove extra day added for february in a leap year
	if (month == 2 && year % 4 == 0) {
		m -= SECONDS_PER_DAY;
	}
	ep -= m;
	tida.da.Month = month;

	//find current day
	tida.da.Date = (ep / SECONDS_PER_DAY) + 1;
	ep %= SECONDS_PER_DAY;

	// find current hour
	tida.ti.Hours = ((ep / SECONDS_PER_HOUR)) % 12;
	if (tida.ti.Hours == 0) {
		tida.ti.Hours = 12;
	}
	tida.ti.TimeFormat = ((ep / SECONDS_PER_HOUR)) / 12;
	ep %= SECONDS_PER_HOUR;

	// find current minute
	tida.ti.Minutes = ep / SECONDS_PER_MINUTE;
	ep %= SECONDS_PER_MINUTE;

	//find current second
	tida.ti.Seconds = ep;

	return tida;

}


#endif
