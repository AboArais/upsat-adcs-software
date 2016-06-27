/*
 * adcs_time.c
 *
 *  Created on: Jun 17, 2016
 *      Author: azisi
 */

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>

#include "adcs_time.h"
#include "adcs_common.h"

time_keeping_adcs adcs_time;

/* Conversion from UTC date to decimal year */
void decyear(time_keeping_adcs *t) {

	int days[] = { 0, 31, 59, 90, 120, 151, 182, 212, 243, 273, 304, 334 };
	int isleap = (((t->year % 4) == 0)
			&& (((t->year % 100) != 0) || ((t->year % 400) == 0)));

	int ndays = isleap ? 366 : 365;
	double day_no = (days[t->month - 1] + t->day + (t->month > 2 ? isleap : 0));
	double day_hour = t->hours + (t->min / 60) + (t->sec / 3600);

	t->decyear = ((double) t->year + (day_no / ndays))
			+ day_hour / SOLAR_DAY_HOURS / ndays;

}

/* Conversion from UTC date to TLE epoch */
void tle_epoch(time_keeping_adcs *t) {

	int days[] = { 0, 31, 59, 90, 120, 151, 182, 212, 243, 273, 304, 334 };
	int isleap = (((t->year % 4) == 0)
			&& (((t->year % 100) != 0) || ((t->year % 400) == 0)));

	t->tle_epoch.ep_year = t->year - 2000;
	t->tle_epoch.ep_day = days[t->month - 1] + t->day
			+ (t->month > 2 ? isleap : 0) + (t->hours / SOLAR_DAY_HOURS)
			+ (t->min / SOLAR_DAY_MIN) + (t->sec / SOLAR_DAY_SEC);

}

/*
 * Conversion from UTC date to Julian days from 1st Jan 1900
 * Reference
 * http://aa.usno.navy.mil/faq/docs/JD_Formula.php
 * Vallado       2007, 189, alg 14, ex 3-14
 */
void julday(time_keeping_adcs *t) {

	float sgn = 100.0 * t->year + t->month - 190002.5;

	if (sgn > 0) { sgn = 1; }
	else if (sgn < 0) {sgn = -1; }
	else { sgn = 0; }

	t->jd = 367.0 * t->year
			- floor((7 * (t->year + floor((t->month + 9) / 12.0))) * 0.25)
			+ floor(275 * t->month / 9.0) + t->day + 1721013.5
			+ ((t->sec / 60.0 + t->min) / 60.0 + t->hours) / 24.0 - 0.5 * sgn
			+ 0.5;

}
