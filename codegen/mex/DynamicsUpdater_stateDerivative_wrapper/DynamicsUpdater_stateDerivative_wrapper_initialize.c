/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * DynamicsUpdater_stateDerivative_wrapper_initialize.c
 *
 * Code generation for function
 * 'DynamicsUpdater_stateDerivative_wrapper_initialize'
 *
 */

/* Include files */
#include "DynamicsUpdater_stateDerivative_wrapper_initialize.h"
#include "DynamicsUpdater_stateDerivative_wrapper.h"
#include "DynamicsUpdater_stateDerivative_wrapper_data.h"
#include "_coder_DynamicsUpdater_stateDerivative_wrapper_mex.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void c_DynamicsUpdater_stateDerivati(void);

/* Function Definitions */
static void c_DynamicsUpdater_stateDerivati(void)
{
  mex_InitInfAndNan();
  d_DynamicsUpdater_stateDerivati();
}

void DynamicsUpdater_stateDerivative_wrapper_initialize(void)
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
    c_DynamicsUpdater_stateDerivati();
  }
}

/* End of code generation (DynamicsUpdater_stateDerivative_wrapper_initialize.c)
 */
