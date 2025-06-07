/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_api.c
 *
 * Code generation for function
 * '_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_api'
 *
 */

/* Include files */
#include "_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_api.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u_data[],
                                          const int32_T u_size[2]);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u_data[],
                                          const int32_T u_size[2])
{
  const mxArray *m;
  const mxArray *y;
  int32_T iv[2];
  y = NULL;
  iv[0] = 1;
  iv[1] = u_size[1];
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, u_size[1], m, &u_data[0]);
  emlrtAssign(&y, m);
  return y;
}

void d_VehicleCollisionSeverity_Calc(int32_T nlhs, const mxArray *plhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T dV1;
  real_T dV2;
  int32_T severity1_size[2];
  int32_T severity2_size[2];
  char_T severity1_data[26];
  char_T severity2_data[26];
  st.tls = emlrtRootTLSGlobal;
  /* Invoke the target function */
  VehicleCollisionSeverity_CalculateSeverity_wrapper(
      &st, &dV1, &dV2, severity1_data, severity1_size, severity2_data,
      severity2_size);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(dV1);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(dV2);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(&st, severity1_data, severity1_size);
  }
  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut(&st, severity2_data, severity2_size);
  }
}

/* End of code generation
 * (_coder_VehicleCollisionSeverity_CalculateSeverity_wrapper_api.c) */
