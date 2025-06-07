/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * error.c
 *
 * Code generation for function 'error'
 *
 */

/* Include files */
#include "error.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo f_emlrtMCI = {
    27,      /* lineNo */
    5,       /* colNo */
    "error", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pName
                                                                       */
};

static emlrtRSInfo qb_emlrtRSI = {
    27,      /* lineNo */
    "error", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pathName
                                                                       */
};

/* Function Declarations */
static void c_error(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location);

/* Function Definitions */
static void c_error(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "error",
                        true, location);
}

void b_error(const emlrtStack *sp, const char_T varargin_2[7])
{
  static const int32_T iv[2] = {1, 86};
  static const int32_T iv1[2] = {1, 7};
  static const char_T varargin_1[86] = {
      'U', 'n',  's',  'u', 'p', 'p', 'o',  'r', 't',  'e',  'd', ' ',  'v',
      'e', 'h',  'i',  'c', 'l', 'e', ' ',  't', 'y',  'p',  'e', ':',  ' ',
      '%', 's',  '.',  ' ', 'U', 's', 'e',  ' ', '\'', 't',  'r', 'a',  'c',
      't', 'o',  'r',  '-', 't', 'r', 'a',  'i', 'l',  'e',  'r', '\'', ',',
      ' ', '\'', 't',  'r', 'a', 'c', 't',  'o', 'r',  '\'', ',', ' ',  'o',
      'r', ' ',  '\'', 'P', 'a', 's', 's',  'e', 'n',  'g',  'e', 'r',  'V',
      'e', 'h',  'i',  'c', 'l', 'e', '\'', '.'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 86, m, &varargin_1[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &varargin_2[0]);
  emlrtAssign(&b_y, m);
  st.site = &qb_emlrtRSI;
  c_error(&st, y, b_y, &f_emlrtMCI);
}

/* End of code generation (error.c) */
