/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_DynamicsUpdater_stateDerivative_wrapper_mex.c
 *
 * Code generation for function
 * '_coder_DynamicsUpdater_stateDerivative_wrapper_mex'
 *
 */

/* Include files */
#include "_coder_DynamicsUpdater_stateDerivative_wrapper_mex.h"
#include "DynamicsUpdater_stateDerivative_wrapper_data.h"
#include "DynamicsUpdater_stateDerivative_wrapper_initialize.h"
#include "DynamicsUpdater_stateDerivative_wrapper_terminate.h"
#include "_coder_DynamicsUpdater_stateDerivative_wrapper_api.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void DynamicsUpdater_stateDerivative_wrapper_mexFunction(int32_T nlhs,
                                                         mxArray *plhs[2],
                                                         int32_T nrhs,
                                                         const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        39, "DynamicsUpdater_stateDerivative_wrapper");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 39,
                        "DynamicsUpdater_stateDerivative_wrapper");
  }
  /* Call the function. */
  e_DynamicsUpdater_stateDerivati(prhs[0], nlhs, outputs);
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
  mexAtExit(&DynamicsUpdater_stateDerivative_wrapper_atexit);
  DynamicsUpdater_stateDerivative_wrapper_initialize();
  DynamicsUpdater_stateDerivative_wrapper_mexFunction(nlhs, plhs, nrhs, prhs);
  DynamicsUpdater_stateDerivative_wrapper_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_DynamicsUpdater_stateDerivative_wrapper_mex.c)
 */
