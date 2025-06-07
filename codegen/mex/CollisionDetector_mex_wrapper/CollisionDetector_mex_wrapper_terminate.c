/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector_mex_wrapper_terminate.c
 *
 * Code generation for function 'CollisionDetector_mex_wrapper_terminate'
 *
 */

/* Include files */
#include "CollisionDetector_mex_wrapper_terminate.h"
#include "CollisionDetector_mex_wrapper_data.h"
#include "_coder_CollisionDetector_mex_wrapper_mex.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void CollisionDetector_mex_wrapper_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025a\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&CollisionDetector_mex_wrapper_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void CollisionDetector_mex_wrapper_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (CollisionDetector_mex_wrapper_terminate.c) */
