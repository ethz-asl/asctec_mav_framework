/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef HELPER_H_
#define HELPER_H_

#include <asctec_hl_comm/mav_ctrl.h>
#include <tf/transform_datatypes.h>

namespace helper
{

/// conversion from AscTec acceleration values to m/s^2
const double ASCTEC_ACC_TO_SI = 9.81e-3;

/// conversion from AscTec turn rates to rad/s
const double ASCTEC_OMEGA_TO_SI = 0.015 * M_PI / 180.0;

/// conversion from AscTec attitude angles to rad
const double ASCTEC_ATTITUDE_TO_SI = 0.01 * M_PI / 180.0;

/// converts AscTec acceleration values to m/s^2
template<typename T>
  inline double asctecAccToSI(const T & val)
  {
    return static_cast<double> (val) * ASCTEC_ACC_TO_SI;
  }

/// converts AscTec turn rates to rad/s
template<typename T>
  inline double asctecOmegaToSI(const T & val)
  {
    return static_cast<double> (val) * ASCTEC_OMEGA_TO_SI;
  }

/// converts AscTec acceleration values to rad
template<typename T>
  inline double asctecAttitudeToSI(const T & val)
  {
    return static_cast<double> (val) * ASCTEC_ATTITUDE_TO_SI;
  }

/// converts a rate in Hz to an integer period in ms.
inline uint16_t rateToPeriod(const double & rate)
{
  if (rate > 0)
    return static_cast<uint16_t> (1000.0 / rate);
  else
    return 0;
}

/// checks if val exceeds [min ... max] and returns a clamped value if necessary
template<typename T>
  inline T clamp(const T & min, const T & max, const T & val)
  {
    if (val >= max)
      return max;
    else if (val <= min)
      return min;
    else
      return val;
  }

/// checks if val exceeds [min ... max] and returns a clamped value if necessary.
template<typename T>
  inline T clamp(const T & min, const T & max, const T & val, bool * clamped)
  {
    if (val >= max)
    {
      *clamped = true;
      return max;
    }
    else if (val <= min)
    {
      *clamped = true;
      return min;
    }
    else
    {
      *clamped = false;
      return val;
    }
  }

/// converts a parameter from double (d) to fixpoint (fp) representation needed by the SSDK running datafusion and position control
/**
 * \f$ fp = d* 2^{15} + 2^{31} \f$; min 0, max \f$2^{32}\f$
 */
inline uint32_t param2Fixpoint(const double & param)
{
  return clamp<uint32_t> (0, 4294967296.0 - 1, param * 32768.0 + 2147483648.0 - 1);
  //                            2^32                    2^15         2^31
}

/// converts a parameter from  fixpoint (fp) representation needed by the SSDK running datafusion and position control to double (d)
/**
 * \f$ d = (p - 2^{31})/2^{15} \f$
 */
inline double param2Double(const uint32_t & param)
{
  return ((double)param - 2147483648.0) / 32768.0;
}

/// converts a debug value fixpoint (fp) representation needed by the SSDK running datafusion and position control to double (d)
/**
 * \f$ d = p/2^{15} \f$
 */
inline double debug2Double(const int16_t & param)
{
  return param * 0.001953125;
  //  return (double)param / 512.0;
}

/// converts the yaw angle and range. AscTec uses 0...360° * 1000, we -pi...+pi
inline int yaw2asctec(const double & yaw)
{
  return ((yaw < 0 ? yaw + 2 * M_PI : yaw) * 180.0 / M_PI) * 1000.0;
}
	
#ifdef __APPLE__
/* Possible method for setting __x87_inline_math__ */
#if (defined(__i386__) || defined(i386) || defined(__amd64) || defined(__x86_64))
#if (!defined(__x87_inline_math__) && defined(__FAST_MATH__))
#define __x87_inline_math__
#endif
#endif

#ifdef __x87_inline_math__
/*
** Compute sine and cosine at same time (faster than separate calls).
** (*s) gets sin(x)
** (*c) gets cos(x)
*/
#define sincos(x,s,c) sincos_x87_inline(x,s,c)
void sincos_x87_inline(double x,double *s,double *c);
extern __inline__  void sincos_x87_inline(double x,double *s,double *c)
    {
    __asm__ ("fsincos;" : "=t" (*c), "=u" (*s) : "0" (x) : "st(7)");
    }
#else
#define sincos(th,x,y) { (*(x))=sin(th); (*(y))=cos(th); }
#endif
#endif

/// converts AscTec's attitude angles to a quaternion
inline void angle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
                             double *y, double *z)
{
  double sR2, cR2, sP2, cP2, sY2, cY2;
  sincos(roll * 0.5, &sR2, &cR2);
  sincos(pitch * 0.5, &sP2, &cP2);
  sincos(yaw * 0.5, &sY2, &cY2);

  // TODO: change rotation order
  // this follows pre- 2012 firmware rotation order: Rz*Rx*Ry
//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

  // Rz*Ry*Rx for 2012 firmware on the LL:
  *w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
  *z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}

template<typename T>
  inline void setDiagonalCovariance(T & covariance_matrix, const double & covariance)
  {
    covariance_matrix[0] = covariance;
    covariance_matrix[4] = covariance;
    covariance_matrix[8] = covariance;
  }

} // end of namespace helper

#endif /* HELPER_H_ */
