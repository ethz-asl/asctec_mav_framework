/*
 * rt_pow32_snf.c
 *
 * Embedded MATLAB Coder code generation for function 'autogen_ekf_propagation'
 *
 * C source code generated on: Wed Aug 24 18:34:48 2011
 *
 */

#include "rt_pow32_snf.h"
#include <math.h>
#include "rt_nonfinite.h"

#ifndef RT_PI
  #define RT_PI      ((real32_T)3.14159265358979323846)
#endif

/* Function: rt_pow32 =====================================================
 * Abstract:
 *   Calls single-precision version of POW, with guard against domain error
 *   and guards against non-finites
 */
real32_T rt_pow32_snf(const real32_T xr, const real32_T yr) 
{
  real32_T axr, ayr;
  if (rtIsNaNF(xr) || rtIsNaNF(yr) ) {
    return( rtNaNF );
  }

  axr = (real32_T)fabs(xr);
  if (rtIsInfF(yr)) {
    if (axr == 1.0F) {
      return( rtNaNF );
    } else if (axr > 1.0F) {
      return( ( yr > 0.0F ) ? rtInfF : 0.0F);
    } else {
      return( ( yr > 0.0F ) ? 0.0F : rtInfF );
    }
  }

  ayr = (real32_T)fabs(yr);
  if (ayr == 0.0F) {
    return( 1.0F );
  } else if (ayr == 1.0F) {
    return( ( yr > 0.0F ) ? xr : 1.0F/xr );
  } else if (yr == 2.0F) {
    return( xr * xr);
  } else if (yr == 0.5F && xr >= 0.0F) {
    return( (real32_T)sqrt(xr) );
  } else if ((xr < 0.0F) && (yr > (real32_T)floor(yr)) ) {
    return( rtNaNF );
  }

  return( (real32_T)pow(xr,yr) );

} /* end rt_pow32_snf */
/* End of Embedded MATLAB Coder code generation (rt_pow32_snf.c) */
