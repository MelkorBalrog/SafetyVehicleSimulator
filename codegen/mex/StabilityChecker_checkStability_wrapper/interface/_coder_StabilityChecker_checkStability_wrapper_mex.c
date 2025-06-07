/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_StabilityChecker_checkStability_wrapper_mex.c
 *
 * Code generation for function
 * '_coder_StabilityChecker_checkStability_wrapper_mex'
 *
 */

/* Include files */
#include "_coder_StabilityChecker_checkStability_wrapper_mex.h"
#include "StabilityChecker_checkStability_wrapper_data.h"
#include "StabilityChecker_checkStability_wrapper_initialize.h"
#include "StabilityChecker_checkStability_wrapper_terminate.h"
#include "_coder_StabilityChecker_checkStability_wrapper_api.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void StabilityChecker_checkStability_wrapper_mexFunction(int32_T nlhs,
                                                         mxArray *plhs[4],
                                                         int32_T nrhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[4];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4,
                        39, "StabilityChecker_checkStability_wrapper");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 39,
                        "StabilityChecker_checkStability_wrapper");
  }
  /* Call the function. */
  f_StabilityChecker_checkStabili(nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  (void)prhs;
  mexAtExit(&StabilityChecker_checkStability_wrapper_atexit);
  StabilityChecker_checkStability_wrapper_initialize();
  StabilityChecker_checkStability_wrapper_mexFunction(nlhs, plhs, nrhs);
  StabilityChecker_checkStability_wrapper_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_StabilityChecker_checkStability_wrapper_mex.c)
 */
