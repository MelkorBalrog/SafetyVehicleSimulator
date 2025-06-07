/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex.c
 *
 * Code generation for function
 * '_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex'
 *
 */

/* Include files */
#include "_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate.h"
#include "_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void VehicleCollisionSeverity_CalculateSeverity_wrapper_mexFunction(
    int32_T nlhs, mxArray *plhs[4], int32_T nrhs)
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
                        50,
                        "VehicleCollisionSeverity_CalculateSeverity_wrapper");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 50,
                        "VehicleCollisionSeverity_CalculateSeverity_wrapper");
  }
  /* Call the function. */
  d_VehicleCollisionSeverity_Calc(nlhs, outputs);
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
  mexAtExit(&VehicleCollisionSeverity_CalculateSeverity_wrapper_atexit);
  VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize();
  VehicleCollisionSeverity_CalculateSeverity_wrapper_mexFunction(nlhs, plhs,
                                                                 nrhs);
  VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation
 * (_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex.c) */
