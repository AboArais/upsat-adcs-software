/*
 * geomag.c
 *
 *  Created on: Apr 26, 2016
 *      Author: azisi
 */

/****************************************************************************/
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     NGDC's Geomagnetic Field Modeling software for the IGRF and WMM      */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Disclaimer: This program has undergone limited testing. It is        */
/*     being distributed unoffically. The National Geophysical Data         */
/*     Center does not guarantee it's correctness.                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Version 7.0:                                                         */
/*     - input file format changed to                                       */
/*            -- accept new DGRF2005 coeffs with 0.01 nT precision          */
/*            -- make sure all values are separated by blanks               */
/*            -- swapped n and m: first is degree, second is order          */
/*     - new my_isnan function improves portablility                        */
/*     - corrected feet to km conversion factor                             */
/*     - fixed date conversion errors for yyyy,mm,dd format                 */
/*     - fixed lon/lat conversion errors for deg,min,sec format             */
/*     - simplified leap year identification                                */
/*     - changed comment: units of ddot and idot are arc-min/yr             */
/*     - added note that this program computes the secular variation as     */
/*            the 1-year difference, rather than the instantaneous change,  */
/*            which can be slightly different                               */
/*     - clarified that height is above ellipsoid, not above mean sea level */
/*            although the difference is negligible for magnetics           */
/*     - changed main(argv,argc) to usual definition main(argc,argv)        */
/*     - corrected rounding of angles close to 60 minutes                   */
/*     Thanks to all who provided bug reports and suggested fixes           */
/*                                                                          */
/*                                          Stefan Maus Jan-25-2010         */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Version 6.1:                                                         */
/*     Included option to read coordinates from a file and output the       */
/*     results to a new file, repeating the input and adding columns        */
/*     for the output                                                       */
/*                                          Stefan Maus Jan-31-2008         */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Version 6.0:                                                         */
/*     Bug fixes for the interpolation between models. Also added warnings  */
/*     for declination at low H and corrected behaviour at geogr. poles.    */
/*     Placed print-out commands into separate routines to facilitate       */
/*     fine-tuning of the tables                                            */
/*                                          Stefan Maus Aug-24-2004         */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*      This program calculates the geomagnetic field values from           */
/*      a spherical harmonic model.  Inputs required by the user are:       */
/*      a spherical harmonic model data file, coordinate preference,        */
/*      altitude, date/range-step, latitude, and longitude.                 */
/*                                                                          */
/*         Spherical Harmonic                                               */
/*         Model Data File       :  Name of the data file containing the    */
/*                                  spherical harmonic coefficients of      */
/*                                  the chosen model.  The model and path   */
/*                                  must be less than PATH chars.           */
/*                                                                          */
/*         Coordinate Preference :  Geodetic (WGS84 latitude and altitude   */
/*                                  above ellipsoid (WGS84),                */
/*                                  or geocentric (spherical, altitude      */
/*                                  measured from the center of the Earth). */
/*                                                                          */
/*         Altitude              :  Altitude above ellipsoid (WGS84). The   */
/*                                  program asks for altitude above mean    */
/*                                  sea level, because the altitude above   */
/*                                  ellipsoid is not known to most users.   */
/*                                  The resulting error is very small and   */
/*                                  negligible for most practical purposes. */
/*                                  If geocentric coordinate preference is  */
/*                                  used, then the altitude must be in the  */
/*                                  range of 6370.20 km - 6971.20 km as     */
/*                                  measured from the center of the earth.  */
/*                                  Enter altitude in kilometers, meters,   */
/*                                  or feet                                 */
/*                                                                          */
/*         Date                  :  Date, in decimal years, for which to    */
/*                                  calculate the values of the magnetic    */
/*                                  field.  The date must be within the     */
/*                                  limits of the model chosen.             */
/*                                                                          */
/*         Latitude              :  Entered in decimal degrees in the       */
/*                                  form xxx.xxx.  Positive for northern    */
/*                                  hemisphere, negative for the southern   */
/*                                  hemisphere.                             */
/*                                                                          */
/*         Longitude             :  Entered in decimal degrees in the       */
/*                                  form xxx.xxx.  Positive for eastern     */
/*                                  hemisphere, negative for the western    */
/*                                  hemisphere.                             */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*      Subroutines called :  degrees_to_decimal,julday,getshc,interpsh,    */
/*                            extrapsh,shval3,dihf,safegets                 */
/*                                                                          */
/****************************************************************************/
#include "geomag.h"

double PREV_SDATE = 0;

int
my_isnan (double d)
{
  return (d != d); /* IEEE: only NaN is not equal to itself */
}

#define GEO_NAN log(-1.0)
#define FT2KM (1.0/0.0003048)
#define PI 3.141592654
#define RAD2DEG (180.0/PI)

#ifndef SEEK_SET
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2
#endif

#define IEXT 0
#define FALSE 0
#define TRUE 1                  /* constants */
#define RECL 81

#define MAXINBUFF RECL+14

/** Max size of in buffer **/

#define MAXREAD MAXINBUFF-2
/** Max to read 2 less than total size (just to be safe) **/

#define EXT_COEFF1 (double)0
#define EXT_COEFF2 (double)0
#define EXT_COEFF3 (double)0

#define MAXDEG 13
#define MAXCOEFF (MAXDEG*(MAXDEG+2)+1) /* index starts with 1!, (from old Fortran?) */
double gh1[MAXCOEFF];
double gh2[MAXCOEFF];
double gha[MAXCOEFF]; /* Geomag global variables */
double ghb[MAXCOEFF];
double d = 0, f = 0, h = 0, i = 0;
double dtemp, ftemp, htemp, itemp;
double x = 0, y = 0, z = 0;
double xtemp, ytemp, ztemp;

/****************************************************************************/
/*                                                                          */
/*                             Program Geomag                               */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*      This program, originally written in FORTRAN, was developed using    */
/*      subroutines written by                                              */
/*      A. Zunde                                                            */
/*      USGS, MS 964, Box 25046 Federal Center, Denver, Co.  80225          */
/*      and                                                                 */
/*      S.R.C. Malin & D.R. Barraclough                                     */
/*      Institute of Geological Sciences, United Kingdom.                   */
/*                                                                          */
/*      Translated                                                          */
/*      into C by    : Craig H. Shaffer                                     */
/*                     29 July, 1988                                        */
/*                                                                          */
/*      Rewritten by : David Owens                                          */
/*                     For Susan McLean                                     */
/*                                                                          */
/*      Maintained by: Stefan Maus                                          */
/*      Contact      : stefan.maus@noaa.gov                                 */
/*                     National Geophysical Data Center                     */
/*                     World Data Center-A for Solid Earth Geophysics       */
/*                     NOAA, E/GC1, 325 Broadway,                           */
/*                     Boulder, CO  80303                                   */
/*                                                                          */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*      Some variables used in this program                                 */
/*                                                                          */
/*    Name         Type                    Usage                            */
/* ------------------------------------------------------------------------ */
/*                                                                          */
/*   a2,b2      Scalar Double          Squares of semi-major and semi-minor */
/*                                     axes of the reference spheroid used  */
/*                                     for transforming between geodetic or */
/*                                     geocentric coordinates.              */
/*                                                                          */
/*   minalt     Double array of MAXMOD Minimum height of model.             */
/*                                                                          */
/*   altmin     Double                 Minimum height of selected model.    */
/*                                                                          */
/*   altmax     Double array of MAXMOD Maximum height of model.             */
/*                                                                          */
/*   maxalt     Double                 Maximum height of selected model.    */
/*                                                                          */
/*   d          Scalar Double          Declination of the field from the    */
/*                                     geographic north (deg).              */
/*                                                                          */
/*   sdate      Scalar Double          start date inputted                  */
/*                                                                          */
/*   ddot       Scalar Double          annual rate of change of decl.       */
/*                                     (arc-min/yr)                         */
/*                                                                          */
/*   alt        Scalar Double          altitude above WGS84 Ellipsoid       */
/*                                                                          */
/*   epoch      Double array of MAXMOD epoch of model.                      */
/*                                                                          */
/*   ext        Scalar Double          Three 1st-degree external coeff.     */
/*                                                                          */
/*   latitude   Scalar Double          Latitude.                            */
/*                                                                          */
/*   longitude  Scalar Double          Longitude.                           */
/*                                                                          */
/*   gh1        Double array           Schmidt quasi-normal internal        */
/*                                     spherical harmonic coeff.            */
/*                                                                          */
/*   gh2        Double array           Schmidt quasi-normal internal        */
/*                                     spherical harmonic coeff.            */
/*                                                                          */
/*   gha        Double array           Coefficients of resulting model.     */
/*                                                                          */
/*   ghb        Double array           Coefficients of rate of change model.*/
/*                                                                          */
/*   i          Scalar Double          Inclination (deg).                   */
/*                                                                          */
/*   idot       Scalar Double          Rate of change of i (arc-min/yr).    */
/*                                                                          */
/*   igdgc      Integer                Flag for geodetic or geocentric      */
/*                                     coordinate choice.                   */
/*                                                                          */
/*   inbuff     Char a of MAXINBUF     Input buffer.                        */
/*                                                                          */
/*   irec_pos   Integer array of MAXMOD Record counter for header           */
/*                                                                          */
/*   stream  Integer                   File handles for an opened file.     */
/*                                                                          */
/*   fileline   Integer                Current line in file (for errors)    */
/*                                                                          */
/*   max1       Integer array of MAXMOD Main field coefficient.             */
/*                                                                          */
/*   max2       Integer array of MAXMOD Secular variation coefficient.      */
/*                                                                          */
/*   max3       Integer array of MAXMOD Acceleration coefficient.           */
/*                                                                          */
/*   mdfile     Character array of PATH  Model file name.                   */
/*                                                                          */
/*   minyr      Double                  Min year of all models              */
/*                                                                          */
/*   maxyr      Double                  Max year of all models              */
/*                                                                          */
/*   yrmax      Double array of MAXMOD  Max year of model.                  */
/*                                                                          */
/*   yrmin      Double array of MAXMOD  Min year of model.                  */
/*                                                                          */
/****************************************************************************/
int
geomag (geomagStruct *gStr)
{
  /*  Variable declaration  */

  double sdate = gStr->sdate;
  double latitude = gStr->latitude;
  double longitude = gStr->longitude;
  double alt = gStr->alt;

  /* Control variables */
  int igdgc = 2; /* GEOCENTRIC */

  static int max1;
  static int max2;
  static int max3;
  static int nmax;

  static uint8_t model[] = "IGRF2015";
  static double epoch;
  static double yrmin;
  static double yrmax;
  static double minyr;
  static double maxyr;
  static double altmin;
  static double altmax;
  static double minalt;
  static double maxalt;

  /*  Obtain the desired model file and read the data  */
  if (PREV_SDATE == 0) {
    epoch = 2015.00;
    max1 = 13;
    max2 = 8;
    max3 = 0;
    yrmin = 2015.00;
    yrmax = 2020.00;
    altmin = -1.0;
    altmax = 600.0;

    minyr = yrmin;
    maxyr = yrmax;
  }

  PREV_SDATE = sdate;
  /* Get altitude min and max for selected model. */
  minalt = altmin;
  maxalt = altmax;
  /* Get Coordinate prefs */
  /* If needed modify ranges to reflect coords. */
  if (igdgc == 2) {
    minalt += 6371.2; /* Add radius to ranges. */
    maxalt += 6371.2;
  }
  /** This will compute everything needed for 1 point in time. **/
  getshc (1, max1, 1);
  getshc (0, max2, 2);
  nmax = extrapsh (sdate, epoch, max1, max2, 3);
  nmax = extrapsh (sdate + 1, epoch, max1, max2, 4);

  /* Do the first calculations */
  shval3 (igdgc, latitude, longitude, alt, nmax, 3, IEXT, EXT_COEFF1,
  EXT_COEFF2,
	  EXT_COEFF3);
  dihf (3);
  shval3 (igdgc, latitude, longitude, alt, nmax, 4, IEXT, EXT_COEFF1,
  EXT_COEFF2,
	  EXT_COEFF3);
  dihf (4);
  d = d * (RAD2DEG);
  i = i * (RAD2DEG);

  /* deal with geographic and magnetic poles */
  /* at magnetic poles */
  if (h < 100.0) {
    d = GEO_NAN;
    /* while rest is ok */
  }
  /* at geographic poles */
  if (90.0 - fabs (latitude) <= 0.001) {
    x = GEO_NAN;
    y = GEO_NAN;
    d = GEO_NAN;
    /* while rest is ok */
  }
  /** Above will compute everything for 1 point in time.  **/
  gStr->Xm = x;
  gStr->Ym = y;
  gStr->Zm = z;
  gStr->decl = d;
  gStr->incl = i;
  gStr->h = h;
  gStr->f = f;

  return 0;
}

/****************************************************************************/
/*                                                                          */
/*                       Subroutine degrees_to_decimal                      */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Converts degrees,minutes, seconds to decimal degrees.                */
/*                                                                          */
/*     Input:                                                               */
/*            degrees - Integer degrees                                     */
/*            minutes - Integer minutes                                     */
/*            seconds - Integer seconds                                     */
/*                                                                          */
/*     Output:                                                              */
/*            decimal - degrees in decimal degrees                          */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 12, 1988                                                */
/*                                                                          */
/****************************************************************************/
double
degrees_to_decimal (int degrees, int minutes, int seconds)
{
  double deg;
  double min;
  double sec;
  double decimal;

  deg = degrees;
  min = minutes / 60.0;
  sec = seconds / 3600.0;

  decimal = fabs (sec) + fabs (min) + fabs (deg);

  if (deg < 0) {
    decimal = -decimal;
  }
  else if (deg == 0) {
    if (min < 0) {
      decimal = -decimal;
    }
    else if (min == 0) {
      if (sec < 0) {
	decimal = -decimal;
      }
    }
  }

  return (decimal);
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine julday                              */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Computes the decimal day of year from month, day, year.              */
/*     Supplied by Daniel Bergstrom                                         */
/*                                                                          */
/* References:                                                              */
/*                                                                          */
/* 1. Nachum Dershowitz and Edward M. Reingold, Calendrical Calculations,   */
/*    Cambridge University Press, 3rd edition, ISBN 978-0-521-88540-9.      */
/*                                                                          */
/* 2. Claus Tøndering, Frequently Asked Questions about Calendars,          */
/*    Version 2.9, http://www.tondering.dk/claus/calendar.html              */
/*                                                                          */
/****************************************************************************/
double julday (month, day, year)
  int month;int day;int year;
{
  int days[12] =
    { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

  int leap_year = (((year % 4) == 0)
      && (((year % 100) != 0) || ((year % 400) == 0)));

  double day_in_year = (days[month - 1] + day + (month > 2 ? leap_year : 0));

  return ((double) year + (day_in_year / (365.0 + leap_year)));
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine getshc                              */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Reads spherical harmonic coefficients from the specified             */
/*     model into an array.                                                 */
/*                                                                          */
/*     Input:                                                               */
/*           stream     - Logical unit number                               */
/*           iflag      - Flag for SV equal to ) or not equal to 0          */
/*                        for designated read statements                    */
/*           strec      - Starting record number to read from model         */
/*           nmax_of_gh - Maximum degree and order of model                 */
/*                                                                          */
/*     Output:                                                              */
/*           gh1 or 2   - Schmidt quasi-normal internal spherical           */
/*                        harmonic coefficients                             */
/*                                                                          */
/*     FORTRAN                                                              */
/*           Bill Flanagan                                                  */
/*           NOAA CORPS, DESDIS, NGDC, 325 Broadway, Boulder CO.  80301     */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 15, 1988                                                */
/*                                                                          */
/****************************************************************************/

int
getshc (int iflag, int nmax_of_gh, int gh)
{
  uint8_t irat[] = "IGRF2015";
  int ii, m, n, mm, nn;
  int ios;
  int line_num;
  double g, hh;
  double trash;

  ii = 0;
  ios = 0;
  line_num = 0;

  for (nn = 1; nn <= nmax_of_gh; ++nn) {
    for (mm = 0; mm <= nn; ++mm) {
      if (iflag == 1) {
	n = nn;
	m = mm;
	g = COEFF1[line_num];
	hh = COEFF2[line_num];
	trash = COEFF3[line_num];
	trash = COEFF4[line_num];
	line_num++;
      }
      else {
	n = nn;
	m = mm;
	trash = COEFF1[line_num];
	trash = COEFF2[line_num];
	g = COEFF3[line_num];
	hh = COEFF4[line_num];
	line_num++;
      }
      if ((nn != n) || (mm != m)) {
	ios = -2;
	return (ios);
      }
      ii = ii + 1;
      switch (gh)
	{
	case 1:
	  gh1[ii] = g;
	  break;
	case 2:
	  gh2[ii] = g;
	  break;
	default:
	  break;
	}
      if (m != 0) {
	ii = ii + 1;
	switch (gh)
	  {
	  case 1:
	    gh1[ii] = hh;
	    break;
	  case 2:
	    gh2[ii] = hh;
	    break;
	  default:
	    break;
	  }
      }
    }
  }

  return (ios);
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine extrapsh                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Extrapolates linearly a spherical harmonic model with a              */
/*     rate-of-change model.                                                */
/*                                                                          */
/*     Input:                                                               */
/*           date     - date of resulting model (in decimal year)           */
/*           dte1     - date of base model                                  */
/*           nmax1    - maximum degree and order of base model              */
/*           gh1      - Schmidt quasi-normal internal spherical             */
/*                      harmonic coefficients of base model                 */
/*           nmax2    - maximum degree and order of rate-of-change model    */
/*           gh2      - Schmidt quasi-normal internal spherical             */
/*                      harmonic coefficients of rate-of-change model       */
/*                                                                          */
/*     Output:                                                              */
/*           gha or b - Schmidt quasi-normal internal spherical             */
/*                    harmonic coefficients                                 */
/*           nmax   - maximum degree and order of resulting model           */
/*                                                                          */
/*     FORTRAN                                                              */
/*           A. Zunde                                                       */
/*           USGS, MS 964, box 25046 Federal Center, Denver, CO.  80225     */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 16, 1988                                                */
/*                                                                          */
/****************************************************************************/
int
extrapsh (double date, double dte1, int nmax1, int nmax2, int gh)
{
  int nmax;
  int k, l;
  int ii;
  double factor;

  factor = date - dte1;
  if (nmax1 == nmax2) {
    k = nmax1 * (nmax1 + 2);
    nmax = nmax1;
  }
  else {
    if (nmax1 > nmax2) {
      k = nmax2 * (nmax2 + 2);
      l = nmax1 * (nmax1 + 2);
      switch (gh)
	{
	case 3:
	  for (ii = k + 1; ii <= l; ++ii) {
	    gha[ii] = gh1[ii];
	  }
	  break;
	case 4:
	  for (ii = k + 1; ii <= l; ++ii) {
	    ghb[ii] = gh1[ii];
	  }
	  break;
	default:
	  break;
	}
      nmax = nmax1;
    }
    else {
      k = nmax1 * (nmax1 + 2);
      l = nmax2 * (nmax2 + 2);
      switch (gh)
	{
	case 3:
	  for (ii = k + 1; ii <= l; ++ii) {
	    gha[ii] = factor * gh2[ii];
	  }
	  break;
	case 4:
	  for (ii = k + 1; ii <= l; ++ii) {
	    ghb[ii] = factor * gh2[ii];
	  }
	  break;
	default:
	  break;
	}
      nmax = nmax2;
    }
  }
  switch (gh)
    {
    case 3:
      for (ii = 1; ii <= k; ++ii) {
	gha[ii] = gh1[ii] + factor * gh2[ii];
      }
      break;
    case 4:
      for (ii = 1; ii <= k; ++ii) {
	ghb[ii] = gh1[ii] + factor * gh2[ii];
      }
      break;
    default:
      break;
    }
  return (nmax);
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine interpsh                            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Interpolates linearly, in time, between two spherical harmonic       */
/*     models.                                                              */
/*                                                                          */
/*     Input:                                                               */
/*           date     - date of resulting model (in decimal year)           */
/*           dte1     - date of earlier model                               */
/*           nmax1    - maximum degree and order of earlier model           */
/*           gh1      - Schmidt quasi-normal internal spherical             */
/*                      harmonic coefficients of earlier model              */
/*           dte2     - date of later model                                 */
/*           nmax2    - maximum degree and order of later model             */
/*           gh2      - Schmidt quasi-normal internal spherical             */
/*                      harmonic coefficients of internal model             */
/*                                                                          */
/*     Output:                                                              */
/*           gha or b - coefficients of resulting model                     */
/*           nmax     - maximum degree and order of resulting model         */
/*                                                                          */
/*     FORTRAN                                                              */
/*           A. Zunde                                                       */
/*           USGS, MS 964, box 25046 Federal Center, Denver, CO.  80225     */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 17, 1988                                                */
/*                                                                          */
/****************************************************************************/
int
interpsh (double date, double dte1, int nmax1, double dte2, int nmax2, int gh)
{
  int nmax;
  int k, l;
  int ii;
  double factor;

  factor = (date - dte1) / (dte2 - dte1);
  if (nmax1 == nmax2) {
    k = nmax1 * (nmax1 + 2);
    nmax = nmax1;
  }
  else {
    if (nmax1 > nmax2) {
      k = nmax2 * (nmax2 + 2);
      l = nmax1 * (nmax1 + 2);
      switch (gh)
	{
	case 3:
	  for (ii = k + 1; ii <= l; ++ii) {
	    gha[ii] = gh1[ii] + factor * (-gh1[ii]);
	  }
	  break;
	case 4:
	  for (ii = k + 1; ii <= l; ++ii) {
	    ghb[ii] = gh1[ii] + factor * (-gh1[ii]);
	  }
	  break;
	default:
	  break;
	}
      nmax = nmax1;
    }
    else {
      k = nmax1 * (nmax1 + 2);
      l = nmax2 * (nmax2 + 2);
      switch (gh)
	{
	case 3:
	  for (ii = k + 1; ii <= l; ++ii) {
	    gha[ii] = factor * gh2[ii];
	  }
	  break;
	case 4:
	  for (ii = k + 1; ii <= l; ++ii) {
	    ghb[ii] = factor * gh2[ii];
	  }
	  break;
	default:
	  break;
	}
      nmax = nmax2;
    }
  }
  switch (gh)
    {
    case 3:
      for (ii = 1; ii <= k; ++ii) {
	gha[ii] = gh1[ii] + factor * (gh2[ii] - gh1[ii]);
      }
      break;
    case 4:
      for (ii = 1; ii <= k; ++ii) {
	ghb[ii] = gh1[ii] + factor * (gh2[ii] - gh1[ii]);
      }
      break;
    default:
      break;
    }
  return (nmax);
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine shval3                              */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Calculates field components from spherical harmonic (sh)             */
/*     models.                                                              */
/*                                                                          */
/*     Input:                                                               */
/*           igdgc     - indicates coordinate system used; set equal        */
/*                       to 1 if geodetic, 2 if geocentric                  */
/*           latitude  - north latitude, in degrees                         */
/*           longitude - east longitude, in degrees                         */
/*           elev      - WGS84 altitude above ellipsoid (igdgc=1), or       */
/*                       radial distance from earth's center (igdgc=2)      */
/*           a2,b2     - squares of semi-major and semi-minor axes of       */
/*                       the reference spheroid used for transforming       */
/*                       between geodetic and geocentric coordinates        */
/*                       or components                                      */
/*           nmax      - maximum degree and order of coefficients           */
/*           iext      - external coefficients flag (=0 if none)            */
/*           ext1,2,3  - the three 1st-degree external coefficients         */
/*                       (not used if iext = 0)                             */
/*                                                                          */
/*     Output:                                                              */
/*           x         - northward component                                */
/*           y         - eastward component                                 */
/*           z         - vertically-downward component                      */
/*                                                                          */
/*     based on subroutine 'igrf' by D. R. Barraclough and S. R. C. Malin,  */
/*     report no. 71/1, institute of geological sciences, U.K.              */
/*                                                                          */
/*     FORTRAN                                                              */
/*           Norman W. Peddie                                               */
/*           USGS, MS 964, box 25046 Federal Center, Denver, CO.  80225     */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 17, 1988                                                */
/*                                                                          */
/****************************************************************************/
int
shval3 (int igdgc, double flat, double flon, double elev, int nmax, int gh,
	int iext, double ext1, double ext2, double ext3)
{
  double earths_radius = 6371.2;
  double dtr = 0.01745329;
  double slat;
  double clat;
  double ratio;
  double aa, bb, cc, dd;
  double sd;
  double cd;
  double r;
  double a2;
  double b2;
  double rr;
  double fm, fn;
  double sl[14];
  double cl[14];
  double p[119];
  double q[119];
  int ii, j, k, l, m, n;
  int npq;
  int ios;
  double argument;
  double power;
  a2 = 40680631.59; /* WGS84 */
  b2 = 40408299.98; /* WGS84 */
  ios = 0;
  r = elev;
  argument = flat * dtr;
  slat = sin (argument);
  if ((90.0 - flat) < 0.001) {
    aa = 89.999; /*  300 ft. from North pole  */
  }
  else {
    if ((90.0 + flat) < 0.001) {
      aa = -89.999; /*  300 ft. from South pole  */
    }
    else {
      aa = flat;
    }
  }
  argument = aa * dtr;
  clat = cos (argument);
  argument = flon * dtr;
  sl[1] = sin (argument);
  cl[1] = cos (argument);
  switch (gh)
    {
    case 3:
      x = 0;
      y = 0;
      z = 0;
      break;
    case 4:
      xtemp = 0;
      ytemp = 0;
      ztemp = 0;
      break;
    default:
      break;
    }
  sd = 0.0;
  cd = 1.0;
  l = 1;
  n = 0;
  m = 1;
  npq = (nmax * (nmax + 3)) / 2;
  if (igdgc == 1) {
    aa = a2 * clat * clat;
    bb = b2 * slat * slat;
    cc = aa + bb;
    argument = cc;
    dd = sqrt (argument);
    argument = elev * (elev + 2.0 * dd) + (a2 * aa + b2 * bb) / cc;
    r = sqrt (argument);
    cd = (elev + dd) / r;
    sd = (a2 - b2) / dd * slat * clat / r;
    aa = slat;
    slat = slat * cd - clat * sd;
    clat = clat * cd + aa * sd;
  }
  ratio = earths_radius / r;
  argument = 3.0;
  aa = sqrt (argument);
  p[1] = 2.0 * slat;
  p[2] = 2.0 * clat;
  p[3] = 4.5 * slat * slat - 1.5;
  p[4] = 3.0 * aa * clat * slat;
  q[1] = -clat;
  q[2] = slat;
  q[3] = -3.0 * clat * slat;
  q[4] = aa * (slat * slat - clat * clat);
  for (k = 1; k <= npq; ++k) {
    if (n < m) {
      m = 0;
      n = n + 1;
      argument = ratio;
      power = n + 2;
      rr = pow (argument, power);
      fn = n;
    }
    fm = m;
    if (k >= 5) {
      if (m == n) {
	argument = (1.0 - 0.5 / fm);
	aa = sqrt (argument);
	j = k - n - 1;
	p[k] = (1.0 + 1.0 / fm) * aa * clat * p[j];
	q[k] = aa * (clat * q[j] + slat / fm * p[j]);
	sl[m] = sl[m - 1] * cl[1] + cl[m - 1] * sl[1];
	cl[m] = cl[m - 1] * cl[1] - sl[m - 1] * sl[1];
      }
      else {
	argument = fn * fn - fm * fm;
	aa = sqrt (argument);
	argument = ((fn - 1.0) * (fn - 1.0)) - (fm * fm);
	bb = sqrt (argument) / aa;
	cc = (2.0 * fn - 1.0) / aa;
	ii = k - n;
	j = k - 2 * n + 1;
	p[k] = (fn + 1.0) * (cc * slat / fn * p[ii] - bb / (fn - 1.0) * p[j]);
	q[k] = cc * (slat * q[ii] - clat / fn * p[ii]) - bb * q[j];
      }
    }
    switch (gh)
      {
      case 3:
	aa = rr * gha[l];
	break;
      case 4:
	aa = rr * ghb[l];
	break;
      default:
	break;
      }
    if (m == 0) {
      switch (gh)
	{
	case 3:
	  x = x + aa * q[k];
	  z = z - aa * p[k];
	  break;
	case 4:
	  xtemp = xtemp + aa * q[k];
	  ztemp = ztemp - aa * p[k];
	  break;
	default:
	  break;
	}
      l = l + 1;
    }
    else {
      switch (gh)
	{
	case 3:
	  bb = rr * gha[l + 1];
	  cc = aa * cl[m] + bb * sl[m];
	  x = x + cc * q[k];
	  z = z - cc * p[k];
	  if (clat > 0) {
	    y = y + (aa * sl[m] - bb * cl[m]) * fm * p[k] / ((fn + 1.0) * clat);
	  }
	  else {
	    y = y + (aa * sl[m] - bb * cl[m]) * q[k] * slat;
	  }
	  l = l + 2;
	  break;
	case 4:
	  bb = rr * ghb[l + 1];
	  cc = aa * cl[m] + bb * sl[m];
	  xtemp = xtemp + cc * q[k];
	  ztemp = ztemp - cc * p[k];
	  if (clat > 0) {
	    ytemp = ytemp
		+ (aa * sl[m] - bb * cl[m]) * fm * p[k] / ((fn + 1.0) * clat);
	  }
	  else {
	    ytemp = ytemp + (aa * sl[m] - bb * cl[m]) * q[k] * slat;
	  }
	  l = l + 2;
	  break;
	default:
	  break;
	}
    }
    m = m + 1;
  }
  if (iext != 0) {
    aa = ext2 * cl[1] + ext3 * sl[1];
    switch (gh)
      {
      case 3:
	x = x - ext1 * clat + aa * slat;
	y = y + ext2 * sl[1] - ext3 * cl[1];
	z = z + ext1 * slat + aa * clat;
	break;
      case 4:
	xtemp = xtemp - ext1 * clat + aa * slat;
	ytemp = ytemp + ext2 * sl[1] - ext3 * cl[1];
	ztemp = ztemp + ext1 * slat + aa * clat;
	break;
      default:
	break;
      }
  }
  switch (gh)
    {
    case 3:
      aa = x;
      x = x * cd + z * sd;
      z = z * cd - aa * sd;
      break;
    case 4:
      aa = xtemp;
      xtemp = xtemp * cd + ztemp * sd;
      ztemp = ztemp * cd - aa * sd;
      break;
    default:
      break;
    }
  return (ios);
}

/****************************************************************************/
/*                                                                          */
/*                           Subroutine dihf                                */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*     Computes the geomagnetic d, i, h, and f from x, y, and z.            */
/*                                                                          */
/*     Input:                                                               */
/*           x  - northward component                                       */
/*           y  - eastward component                                        */
/*           z  - vertically-downward component                             */
/*                                                                          */
/*     Output:                                                              */
/*           d  - declination                                               */
/*           i  - inclination                                               */
/*           h  - horizontal intensity                                      */
/*           f  - total intensity                                           */
/*                                                                          */
/*     FORTRAN                                                              */
/*           A. Zunde                                                       */
/*           USGS, MS 964, box 25046 Federal Center, Denver, CO.  80225     */
/*                                                                          */
/*     C                                                                    */
/*           C. H. Shaffer                                                  */
/*           Lockheed Missiles and Space Company, Sunnyvale CA              */
/*           August 22, 1988                                                */
/*                                                                          */
/****************************************************************************/
int
dihf (int gh)
{
  int ios;
  int j;
  double sn;
  double h2;
  double hpx;
  double argument, argument2;

  ios = gh;
  sn = 0.0001;

  switch (gh)
    {
    case 3:
      for (j = 1; j <= 1; ++j) {
	h2 = x * x + y * y;
	argument = h2;
	h = sqrt (argument); /* calculate horizontal intensity */
	argument = h2 + z * z;
	f = sqrt (argument); /* calculate total intensity */
	if (f < sn) {
	  d = GEO_NAN; /* If d and i cannot be determined, */
	  i = GEO_NAN; /*       set equal to GEO_NAN         */
	}
	else {
	  argument = z;
	  argument2 = h;
	  i = atan2 (argument, argument2);
	  if (h < sn) {
	    d = GEO_NAN;
	  }
	  else {
	    hpx = h + x;
	    if (hpx < sn) {
	      d = PI;
	    }
	    else {
	      argument = y;
	      argument2 = hpx;
	      d = 2.0 * atan2 (argument, argument2);
	    }
	  }
	}
      }
      break;
    case 4:
      for (j = 1; j <= 1; ++j) {
	h2 = xtemp * xtemp + ytemp * ytemp;
	argument = h2;
	htemp = sqrt (argument);
	argument = h2 + ztemp * ztemp;
	ftemp = sqrt (argument);
	if (ftemp < sn) {
	  dtemp = GEO_NAN; /* If d and i cannot be determined, */
	  itemp = GEO_NAN; /*       set equal to 999.0         */
	}
	else {
	  argument = ztemp;
	  argument2 = htemp;
	  itemp = atan2 (argument, argument2);
	  if (htemp < sn) {
	    dtemp = GEO_NAN;
	  }
	  else {
	    hpx = htemp + xtemp;
	    if (hpx < sn) {
	      dtemp = PI;
	    }
	    else {
	      argument = ytemp;
	      argument2 = hpx;
	      dtemp = 2.0 * atan2 (argument, argument2);
	    }
	  }
	}
      }
      break;
    default:
      break;
    }
  return (ios);
}

/* */
void
NED2ECEF (geomagStruct *gStr)
{
  double u, n, e, cLat, sLat, cLon, sLon, tmp;
  n = gStr->Xm;
  e = gStr->Ym;
  u = -gStr->Zm;

  cLat = cos (gStr->latitude * PI / 180.0);
  sLat = sin (gStr->latitude * PI / 180.0);
  cLon = cos (gStr->longitude * PI / 180.0);
  sLon = sin (gStr->longitude * PI / 180.0);

  tmp = cLat * u - sLat * n;
  gStr->Zm = sLat * u + cLat * n;
  gStr->Xm = cLon * tmp - sLon * e;
  gStr->Ym = sLon * tmp + cLon * e;
}

/*
 * http://aa.usno.navy.mil/faq/docs/JD_Formula.php
 ---COMPUTES THE GREGORIAN CALENDAR DATE (YEAR,MONTH,DAY)
 GIVEN THE JULIAN DATE (JD).
 ADAPTED for fractional part -UT time as per
 http://quasar.as.utexas.edu/BillInfo/JulianDatesG.html
 */

/* Conversion from a Julian date to a Gregorian calendar date
   and Universal Time */
void
JD2Greg (double JD, gTime *t)
{
  // Volatile for compiler optim
  volatile int iJD, L, N, I, J, K;
  iJD = (int) (JD + 0.5); //for fractional part
  L = iJD + 68569;

  N = (4 * L / 146097);
  L = (L - (146097 * N + 3) / 4);
  I = (4000 * (L + 1) / 1461001);
  L = (L - 1461 * I / 4 + 31);
  J = (80 * L / 2447);
  K = (L - 2447 * J / 80);
  L = (J / 11);
  J = (J + 2 - 12 * L);
  I = 100 * (N - 49) + I + L;
  t->year = I;
  t->month = J;
  t->day = K;
  t->UT = (JD - iJD + 0.5) * 24;
}

/* Conversion from a Gregorian calendar date to a Julian date */
double
Greg2JD (gTime t)
{
  double UT = t.UT;
  UT = UT - 12;
  return t.day - 32075 + 1461 * (t.year + 4800 + (t.month - 14) / 12) / 4
      + 367 * (t.month - 2 - (t.month - 14) / 12 * 12) / 12
      - 3 * ((t.year + 4900 + (t.month - 14) / 12) / 100) / 4 + UT / 24.0;
}

/* Conversion from a Gregorian calendar date to decimal year */
double
decyear (gTime t)
{

  int days[] =
    { 0, 31, 59, 90, 120, 151, 182, 212, 243, 273, 304, 334 };
  int isleap = (((t.year % 4) == 0)
      && (((t.year % 100) != 0) || ((t.year % 400) == 0)));
  int ndays = isleap ? 366 : 365;

  double dayno = (days[t.month - 1] + t.day + (t.month > 2 ? isleap : 0)) - 1;

  return ((double) t.year + (dayno / ndays)) + t.UT / (ndays) / 24.0;
}
