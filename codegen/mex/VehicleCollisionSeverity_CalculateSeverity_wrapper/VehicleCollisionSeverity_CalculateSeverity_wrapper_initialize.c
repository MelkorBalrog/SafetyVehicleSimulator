/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize.c
 *
 * Code generation for function
 * 'VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize'
 *
 */

/* Include files */
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize.h"
#include "VehicleCollisionSeverity.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void c_VehicleCollisionSeverity_Calc(void);

/* Function Definitions */
static void c_VehicleCollisionSeverity_Calc(void)
{
  mex_InitInfAndNan();
  d_VehicleCollisionSeverity_get_();
}

void VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize(void)
{
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
    c_VehicleCollisionSeverity_Calc();
  }
}

/* End of code generation
 * (VehicleCollisionSeverity_CalculateSeverity_wrapper_initialize.c) */
