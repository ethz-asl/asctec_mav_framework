/*
 * autogen_ekf_propagation.h
 *
 * Embedded MATLAB Coder code generation for function 'autogen_ekf_propagation'
 *
 * C source code generated on: Tue Aug 16 17:54:01 2011
 *
 */

#ifndef __AUTOGEN_EKF_PROPAGATION_H__
#define __AUTOGEN_EKF_PROPAGATION_H__
/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rt_pow32_snf.h"

#include "rtwtypes.h"
#include "pos_ekf_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void autogen_ekf_propagation(const real32_T eml_in1[20], const real32_T eml_in2[3], const real32_T eml_in3[3], real32_T eml_dt, real32_T eml_x_new[16]);
#endif
/* End of Embedded MATLAB Coder code generation (autogen_ekf_propagation.h) */
