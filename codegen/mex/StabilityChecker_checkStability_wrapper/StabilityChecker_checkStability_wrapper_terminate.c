/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * StabilityChecker_checkStability_wrapper_terminate.c
 *
 * Code generation for function
 * 'StabilityChecker_checkStability_wrapper_terminate'
 *
 */

/* Include files */
#include "StabilityChecker_checkStability_wrapper_terminate.h"
#include "StabilityChecker_checkStability_wrapper_data.h"
#include "_coder_StabilityChecker_checkStability_wrapper_mex.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void e_StabilityChecker_checkStabili(const void *r);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void e_StabilityChecker_checkStabili(const void *r)
{
}

static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void StabilityChecker_checkStability_wrapper_atexit(void)
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
  emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                    (void *)&e_StabilityChecker_checkStabili,
                                    NULL, NULL, NULL);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void StabilityChecker_checkStability_wrapper_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (StabilityChecker_checkStability_wrapper_terminate.c)
 */
