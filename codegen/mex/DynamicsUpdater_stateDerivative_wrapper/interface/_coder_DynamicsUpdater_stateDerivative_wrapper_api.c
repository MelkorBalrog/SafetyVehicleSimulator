/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_DynamicsUpdater_stateDerivative_wrapper_api.c
 *
 * Code generation for function
 * '_coder_DynamicsUpdater_stateDerivative_wrapper_api'
 *
 */

/* Include files */
#include "_coder_DynamicsUpdater_stateDerivative_wrapper_api.h"
#include "DynamicsUpdater_stateDerivative_wrapper.h"
#include "DynamicsUpdater_stateDerivative_wrapper_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static const mxArray *c_emlrt_marshallOut(real_T u[8]);

static const mxArray *d_emlrt_marshallOut(real_T u[2]);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[8];

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8];

/* Function Definitions */
static const mxArray *c_emlrt_marshallOut(real_T u[8])
{
  static const int32_T i = 0;
  static const int32_T i1 = 8;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static const mxArray *d_emlrt_marshallOut(real_T u[2])
{
  static const int32_T i = 0;
  static const int32_T i1 = 2;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[8]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[8];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8]
{
  real_T(*y)[8];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8]
{
  static const int32_T dims = 8;
  real_T(*ret)[8];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[8])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void e_DynamicsUpdater_stateDerivati(const mxArray *prhs, int32_T nlhs,
                                     const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*dydt)[8];
  real_T(*stateVec)[8];
  real_T(*accelerations)[2];
  st.tls = emlrtRootTLSGlobal;
  dydt = (real_T(*)[8])mxMalloc(sizeof(real_T[8]));
  accelerations = (real_T(*)[2])mxMalloc(sizeof(real_T[2]));
  /* Marshall function inputs */
  stateVec = e_emlrt_marshallIn(&st, emlrtAlias(prhs), "stateVec");
  /* Invoke the target function */
  DynamicsUpdater_stateDerivative_wrapper(&st, *stateVec, *dydt,
                                          *accelerations);
  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*dydt);
  if (nlhs > 1) {
    plhs[1] = d_emlrt_marshallOut(*accelerations);
  }
}

/* End of code generation (_coder_DynamicsUpdater_stateDerivative_wrapper_api.c)
 */
