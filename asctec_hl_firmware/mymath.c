#include "mymath.h"
#include <math.h>

// sin/cos code from http://www.ganssle.com/approx/approx.pdf

// cos_52s computes cosine (x)
//
// Accurate to about 5.2 decimal digits over the range [0, pi/2].
// The input argument is in radians.
//
// Algorithm:
// cos(x)= c1 + c2*x**2 + c3*x**4 + c4*x**6
// which is the same as:
// cos(x)= c1 + x**2(c2 + c3*x**2 + c4*x**4)
// cos(x)= c1 + x**2(c2 + x**2(c3 + c4*x**2))
//

float cos_52s(float x);


float cos_52s(float x)
{
const float c1 = 0.9999932946;
const float c2 = -0.4999124376;
const float c3 = 0.0414877472;
const float c4 = -0.0012712095;
float x2; // The input argument squared
x2 = x * x;
return (c1 + x2 * (c2 + x2 * (c3 + c4 * x2)));
}


//
// This is the main cosine approximation "driver"
// It reduces the input argument's range to [0, pi/2],
// and then calls the approximator.
//
float approxCos(float x)
{
  int quad; // what quadrant are we in?
  x = fmod(x, M_2PI); // Get rid of values > 2* pi
  if (x < 0)
    x = -x; // cos(-x) = cos(x)
  quad=(int)(x/ M_halfPI); // Get quadrant # (0 to 3)
  switch (quad)
  {
    case 0:
      return cos_52s(x);
    case 1:
      return -cos_52s(M_PI - x);
    case 2:
      return -cos_52s(x - M_PI);
    case 3:
      return cos_52s(M_2PI - x);
  }
}


