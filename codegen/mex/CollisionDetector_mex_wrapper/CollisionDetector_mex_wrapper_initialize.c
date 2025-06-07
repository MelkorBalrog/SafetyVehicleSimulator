/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector_mex_wrapper_initialize.c
 *
 * Code generation for function 'CollisionDetector_mex_wrapper_initialize'
 *
 */

/* Include files */
#include "CollisionDetector_mex_wrapper_initialize.h"
#include "CollisionDetector_mex_wrapper_data.h"
#include "_coder_CollisionDetector_mex_wrapper_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void CollisionDetector_mex_wrapper_once(void);

/* Function Definitions */
static void CollisionDetector_mex_wrapper_once(void)
{
  mex_InitInfAndNan();
}

void CollisionDetector_mex_wrapper_initialize(void)
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
    CollisionDetector_mex_wrapper_once();
  }
}

/* End of code generation (CollisionDetector_mex_wrapper_initialize.c) */
