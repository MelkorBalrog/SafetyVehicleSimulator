/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_StabilityChecker_checkStability_wrapper_api.c
 *
 * Code generation for function
 * '_coder_StabilityChecker_checkStability_wrapper_api'
 *
 */

/* Include files */
#include "_coder_StabilityChecker_checkStability_wrapper_api.h"
#include "StabilityChecker_checkStability_wrapper.h"
#include "StabilityChecker_checkStability_wrapper_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static const mxArray *e_emlrt_marshallOut(const boolean_T u);

/* Function Definitions */
static const mxArray *e_emlrt_marshallOut(const boolean_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m);
  return y;
}

void f_StabilityChecker_checkStabili(int32_T nlhs, const mxArray *plhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  boolean_T isJackknife;
  boolean_T isRollover;
  boolean_T isSkidding;
  boolean_T isWiggling;
  st.tls = emlrtRootTLSGlobal;
  /* Invoke the target function */
  StabilityChecker_checkStability_wrapper(&st, &isWiggling, &isRollover,
                                          &isSkidding, &isJackknife);
  /* Marshall function outputs */
  plhs[0] = e_emlrt_marshallOut(isWiggling);
  if (nlhs > 1) {
    plhs[1] = e_emlrt_marshallOut(isRollover);
  }
  if (nlhs > 2) {
    plhs[2] = e_emlrt_marshallOut(isSkidding);
  }
  if (nlhs > 3) {
    plhs[3] = e_emlrt_marshallOut(isJackknife);
  }
}

/* End of code generation (_coder_StabilityChecker_checkStability_wrapper_api.c)
 */
