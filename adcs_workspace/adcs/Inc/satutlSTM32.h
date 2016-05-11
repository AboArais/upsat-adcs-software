/* > satutl.h
 *
 *	(c) Paul Crawford & Andrew Brooks 1994-2010
 *	University of Dundee
 *	psc (at) sat.dundee.ac.uk
 *	arb (at) sat.dundee.ac.uk
 *
 *	Released under the terms of the GNU LGPL V3
 *	http://www.gnu.org/licenses/lgpl-3.0.html
 *	
 *	This software is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 * 
 */

#ifndef _SATUTLSTM32_H
#define _SATUTLSTM32_H

#define ST_SIZE 256

#ifdef __cplusplus
extern "C" {
#endif

/** satutlSTM32.c **/
void *vector(size_t num, size_t size);

/** aries.c **/
double gha_aries(double jd);

/** ferror.c **/
void fatal_error(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* _SATUTLSTM32_H */
