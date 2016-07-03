/*
 * adcs_time.h
 *
 *  Created on: Jun 17, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_TIME_H_
#define INC_ADCS_TIME_H_

#include "time_management_service.h"

typedef struct {
	int ep_year; /* Year of epoch, e.g. 94 for 1994, 100 for 2000AD */
	double ep_day; /* Day of epoch from 00:00 Jan 1st ( = 1.0 ) */
} _tle_epoch;

struct time_utc utc;

typedef struct {
	struct time_utc utc;

	_tle_epoch tle_epoch; // TLE epoch
	double decyear; // Decimal year, for IGRF
	double jd; // Julian days from 1st Jan 1900, for SGP4
} time_keeping_adcs;

extern time_keeping_adcs adcs_time;

void tle_epoch(time_keeping_adcs *t);
void decyear(time_keeping_adcs *t);
void julday(time_keeping_adcs *t);

#endif /* INC_ADCS_TIME_H_ */
