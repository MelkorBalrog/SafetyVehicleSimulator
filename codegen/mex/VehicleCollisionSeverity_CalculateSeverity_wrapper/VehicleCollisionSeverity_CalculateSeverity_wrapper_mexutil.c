/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil.c
 *
 * Code generation for function
 * 'VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil'
 *
 */

/* Include files */
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/* End of code generation
 * (VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil.c) */
