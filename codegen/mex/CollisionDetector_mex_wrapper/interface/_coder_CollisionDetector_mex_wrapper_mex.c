/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_CollisionDetector_mex_wrapper_mex.c
 *
 * Code generation for function '_coder_CollisionDetector_mex_wrapper_mex'
 *
 */

/* Include files */
#include "_coder_CollisionDetector_mex_wrapper_mex.h"
#include "CollisionDetector_mex_wrapper.h"
#include "CollisionDetector_mex_wrapper_data.h"
#include "CollisionDetector_mex_wrapper_initialize.h"
#include "CollisionDetector_mex_wrapper_terminate.h"
#include "_coder_CollisionDetector_mex_wrapper_api.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
void CollisionDetector_mex_wrapper_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                               int32_T nrhs,
                                               const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        29, "CollisionDetector_mex_wrapper");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 29,
                        "CollisionDetector_mex_wrapper");
  }
  /* Call the function. */
  c_CollisionDetector_mex_wrapper(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&CollisionDetector_mex_wrapper_atexit);
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025a\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
  CollisionDetector_mex_wrapper_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    CollisionDetector_mex_wrapper_mexFunction(nlhs, plhs, nrhs, prhs);
    CollisionDetector_mex_wrapper_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_CollisionDetector_mex_wrapper_mex.c) */
