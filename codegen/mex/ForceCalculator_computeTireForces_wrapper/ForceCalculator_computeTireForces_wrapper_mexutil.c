/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * ForceCalculator_computeTireForces_wrapper_mexutil.c
 *
 * Code generation for function
 * 'ForceCalculator_computeTireForces_wrapper_mexutil'
 *
 */

/* Include files */
#include "ForceCalculator_computeTireForces_wrapper_mexutil.h"
#include "rt_nonfinite.h"
#include <string.h>

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

/* End of code generation (ForceCalculator_computeTireForces_wrapper_mexutil.c)
 */
