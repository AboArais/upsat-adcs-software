/*
 * adcs_state.h
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_STATE_H_
#define INC_ADCS_STATE_H_

#include "adcs_configuration.h"
#include "gps.h"
//#include "arm_math.h"

/* SGP4 orbit propagator */
#include "sgdp4h.h"
#include "sgp4ext.h"
/* Geomagnetic Field Model */
#include "geomag.h"

typedef struct
{
  int16_t gyr_raw[3];
  float gyr[3];
  float calib_gyr[3];

  int32_t rm_raw[3];
  float rm_mag[3]; /* in uΤ */
  float rm_gain;

  uint16_t temp_raw;
  float temp_c;

  uint16_t v_sun_raw[5];
  float v_sun[5];
  float long_rough;
  float lat_rough;
  float long_sun;
  float lat_sun;
  /* GPS */
  /* SGDP4 */
  xyz_t p_ECI, v_ECI;
  orbit_t orb_tle;
  xyz_t p_ECEF;
  llh_t p_ECEF_LLH;
  double jd;
  /* Geomag */
  geomagStruct mag_ECEF;
  gTime gen_time;
} _adcs_state;

/* Init ADCS state */
void
init_sens (volatile _adcs_state *state);
void
init_lsm9ds0 (volatile _adcs_state *state);
void
init_rm3100 (volatile _adcs_state *state);
void
init_adt7420 (volatile _adcs_state *state);
void
init_sun_sensor (volatile _adcs_state *state);

/* Update ADCS state */
void
update_sens (volatile _adcs_state *state);
void
update_lsm9ds0 (volatile _adcs_state *state);
void
update_rm3100 (volatile _adcs_state *state);
void
update_adt7420 (volatile _adcs_state *state);
uint16_t
update_ad7682 (uint8_t ch);
void
update_sun_sensor (volatile _adcs_state *state);

/* TLE Update sgp4 propagator, geomagnetic field */
void
calculate_tle (volatile _adcs_state *state);
void
update_tle (volatile _adcs_state *state);
void
update_sgdp4 (volatile _adcs_state *state);
void
update_geomag (volatile _adcs_state *state);
#endif /* INC_ADCS_STATE_H_ */
