/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate.c
 *
 * Code generation for function
 * 'VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate'
 *
 */

/* Include files */
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void VehicleCollisionSeverity_CalculateSeverity_wrapper_atexit(void)
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

void VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation
 * (VehicleCollisionSeverity_CalculateSeverity_wrapper_terminate.c) */
