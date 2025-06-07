/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * StabilityChecker_checkStability_wrapper_initialize.c
 *
 * Code generation for function
 * 'StabilityChecker_checkStability_wrapper_initialize'
 *
 */

/* Include files */
#include "StabilityChecker_checkStability_wrapper_initialize.h"
#include "StabilityChecker_checkStability_wrapper.h"
#include "StabilityChecker_checkStability_wrapper_data.h"
#include "_coder_StabilityChecker_checkStability_wrapper_mex.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void c_StabilityChecker_checkStabili(void);

/* Function Definitions */
static void c_StabilityChecker_checkStabili(void)
{
  mex_InitInfAndNan();
  d_StabilityChecker_checkStabili();
}

void StabilityChecker_checkStability_wrapper_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    c_StabilityChecker_checkStabili();
  }
}

/* End of code generation (StabilityChecker_checkStability_wrapper_initialize.c)
 */
