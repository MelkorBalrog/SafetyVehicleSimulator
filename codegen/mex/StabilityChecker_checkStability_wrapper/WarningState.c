/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * WarningState.c
 *
 * Code generation for function 'WarningState'
 *
 */

/* Include files */
#include "WarningState.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = {
    84,                         /* lineNo */
    21,                         /* colNo */
    "WarningState/callWarning", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pName */
};

static emlrtRSInfo pb_emlrtRSI = {
    84,                         /* lineNo */
    "WarningState/callWarning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pathName */
};

/* Function Declarations */
static void feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                  emlrtMCInfo *location);

/* Function Definitions */
static void feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                  emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "feval",
                        true, location);
}

void WarningState_callWarning(const emlrtStack *sp)
{
  static const int32_T iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 42};
  static const char_T varargin_1[42] = {
      'W', 'i', 'n', 'd', ' ', 'v', 'e', 'c', 't', 'o', 'r', ' ', 'n', 'o',
      't', ' ', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'd', '.', ' ', 'D', 'e',
      'f', 'a', 'u', 'l', 't', ' ', '[', '0', ';', '0', ';', '0', ']', '.'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 42, m, &varargin_1[0]);
  emlrtAssign(&b_y, m);
  st.site = &pb_emlrtRSI;
  feval(&st, y, b_y, &emlrtMCI);
}

/* End of code generation (WarningState.c) */
