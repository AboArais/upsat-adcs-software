/* > satutl.c
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

#include "sgdp4h.h"

#include <ctype.h>

static char *
st_start (char *buf);
static long
i_read (char *str, int start, int stop);
static double
d_read (char *str, int start, int stop);

/* ==================================================================
 Locate the first non-white space character, return location.
 ================================================================== */

static char *
st_start (char *buf)
{
  if (buf == NULL)
    return buf;

  while (*buf != '\0' && isspace(*buf))
    buf++;

  return buf;
}

/* ==================================================================
 Mimick the FORTRAN formatted read (assumes array starts at 1), copy
 characters to buffer then convert.
 ================================================================== */

static long
i_read (char *str, int start, int stop)
{
  long itmp = 0;
  char *buf, *tmp;
  int ii;

  start--; /* 'C' arrays start at 0 */
  stop--;

  tmp = buf = (char *) vector (stop - start + 2, sizeof(char));

  for (ii = start; ii <= stop; ii++) {
    *tmp++ = str[ii]; /* Copy the characters. */
  }
  *tmp = '\0'; /* NUL terminate */

  itmp = atol (buf); /* Convert to long integer. */
  free (buf);

  return itmp;
}

/* ==================================================================
 Mimick the FORTRAN formatted read (assumes array starts at 1), copy
 characters to buffer then convert.
 ================================================================== */

static double
d_read (char *str, int start, int stop)
{
  double dtmp = 0;
  char *buf, *tmp;
  int ii;

  start--;
  stop--;

  tmp = buf = (char *) vector (stop - start + 2, sizeof(char));

  for (ii = start; ii <= stop; ii++) {
    *tmp++ = str[ii]; /* Copy the characters. */
  }
  *tmp = '\0'; /* NUL terminate */

  dtmp = atof (buf); /* Convert to long integer. */
  free (buf);

  return dtmp;
}

/* ==================================================================
 Allocate and check an all-zero array of memory (storage vector).
 ================================================================== */

void *
vector (size_t num, size_t size)
{
  void *ptr;

  ptr = calloc (num, size);
  if (ptr == NULL) {
    ; //fatal_error("vector: Allocation failed %u * %u", num, size);
  }

  return ptr;
}

/* ====================================================================== */
