
/*
 * sun_pos.h
 *
 *  Created on: May 23, 2016
 *      Author: azisi
 */

#ifndef INC_SUN_POS_H_
#define INC_SUN_POS_H_

#define AU2KM	149597871
#define PI	3.141592654
#define TWOPI	6.283185308
#define HALFPI	1.570796327
#define DEG2RAD	0.017453293
#define RAD2DEG	57.295779506

void
sun (double jd, double *rsun, double *rtasc, double *decl);

#endif /* INC_SUN_POS_H_ */
