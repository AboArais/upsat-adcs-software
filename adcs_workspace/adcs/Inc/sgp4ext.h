#ifndef INC_SGP4EXT_H_
#define INC_SGP4EXT_H_

/*     ----------------------------------------------------------------
 *
 *                                 sgp4ext.h
 *
 *    this file contains extra routines needed for the main test program for sgp4.
 *    these routines are derived from the astro libraries.
 *
 *                            companion code for
 *               fundamentals of astrodynamics and applications
 *                                    2007
 *                              by david vallado
 *
 *       (w) 719-573-2600, email dvallado@agi.com
 *
 *    current :
 *              20 apr 07  david vallado
 *                           misc documentation updates
 *    changes :
 *              14 aug 06  david vallado
 *                           original baseline
 *       ----------------------------------------------------------------      */

#include <string.h>
#include <math.h> /* change to arm_math.h */
#include "sgdp4h.h"

/** ------------------------ function declarations ------------------------ **/

double
sgn (double x);

double
mag (double x[3]);

void
cross (double vec1[3], double vec2[3], double outvec[3]);

double
dot (double x[3], double y[3]);

double
angle (double vec1[3], double vec2[3]);

void
newtonnu (double ecc, double nu, double *e0, double *m);

double
asinh (double xval);

void
rv2coe (double r[3], double v[3], double mu, double *p, double *a, double *ecc,
    double *incl, double *omega, double *argp, double *nu, double *m,
    double *arglat, double *truelon, double *lonper);

#endif /* INC_SGP4EXT_H_ */
