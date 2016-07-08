/*
 * geomag.h
 *
 *  Created on: Aug 12, 2014
 Author: mobintu
 */
#ifndef GEOMAG_H_
#define GEOMAG_H_

typedef struct {
	double sdate;
	double latitude;
	double longitude;
	double alt;
	double Xm;
	double Ym;
	double Zm;
	double norm;
	double decl;
	double incl;
	double h;
	double f;
} geomag_vector;

extern geomag_vector igrf_vector;

void init_geomag(geomag_vector *gStr);
uint8_t geomag(geomag_vector *gStr);

#endif /* GEOMAG_H_ */
