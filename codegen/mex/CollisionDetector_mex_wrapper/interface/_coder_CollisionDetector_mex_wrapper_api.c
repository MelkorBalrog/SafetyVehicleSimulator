/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_CollisionDetector_mex_wrapper_api.c
 *
 * Code generation for function '_coder_CollisionDetector_mex_wrapper_api'
 *
 */

/* Include files */
#include "_coder_CollisionDetector_mex_wrapper_api.h"
#include "CollisionDetector_mex_wrapper.h"
#include "CollisionDetector_mex_wrapper_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[8];

static const mxArray *emlrt_marshallOut(const boolean_T u);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8]
{
  real_T(*y)[8];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8]
{
  static const int32_T dims[2] = {4, 2};
  real_T(*ret)[8];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[8])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[8]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[8];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(const boolean_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m);
  return y;
}

void c_CollisionDetector_mex_wrapper(const mxArray *const prhs[2],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*corners1)[8];
  real_T(*corners2)[8];
  boolean_T collision;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  corners1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "corners1");
  corners2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "corners2");
  /* Invoke the target function */
  collision = CollisionDetector_mex_wrapper(&st, *corners1, *corners2);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(collision);
}

/* End of code generation (_coder_CollisionDetector_mex_wrapper_api.c) */
