/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * ForceCalculator_computeTireForces_wrapper_terminate.c
 *
 * Code generation for function
 * 'ForceCalculator_computeTireForces_wrapper_terminate'
 *
 */

/* Include files */
#include "ForceCalculator_computeTireForces_wrapper_terminate.h"
#include "ForceCalculator_computeTireForces_wrapper_data.h"
#include "_coder_ForceCalculator_computeTireForces_wrapper_mex.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void ForceCalculator_computeTireForces_wrapper_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void ForceCalculator_computeTireForces_wrapper_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation
 * (ForceCalculator_computeTireForces_wrapper_terminate.c) */
