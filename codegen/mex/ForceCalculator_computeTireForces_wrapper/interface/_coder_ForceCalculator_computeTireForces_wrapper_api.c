/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_ForceCalculator_computeTireForces_wrapper_api.c
 *
 * Code generation for function
 * '_coder_ForceCalculator_computeTireForces_wrapper_api'
 *
 */

/* Include files */
#include "_coder_ForceCalculator_computeTireForces_wrapper_api.h"
#include "ForceCalculator_computeTireForces_wrapper.h"
#include "ForceCalculator_computeTireForces_wrapper_data.h"
#include "ForceCalculator_computeTireForces_wrapper_mexutil.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[4];

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims = 4;
  real_T(*ret)[4];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void e_ForceCalculator_computeTireFo(const mxArray *const prhs[5], int32_T nlhs,
                                     const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*contactAreas)[4];
  real_T(*loads)[4];
  real_T F_y_total;
  real_T M_z;
  real_T r;
  real_T u;
  real_T v;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  loads = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "loads");
  contactAreas = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "contactAreas");
  u = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "u");
  v = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "v");
  r = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "r");
  /* Invoke the target function */
  ForceCalculator_computeTireForces_wrapper(&st, *loads, *contactAreas, u, v, r,
                                            &F_y_total, &M_z);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(F_y_total);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(M_z);
  }
}

/* End of code generation
 * (_coder_ForceCalculator_computeTireForces_wrapper_api.c) */
