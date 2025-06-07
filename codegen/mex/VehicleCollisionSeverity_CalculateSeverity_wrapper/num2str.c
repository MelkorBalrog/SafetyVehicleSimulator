/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * num2str.c
 *
 * Code generation for function 'num2str'
 *
 */

/* Include files */
#include "num2str.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_mexutil.h"
#include "fixEncoding.h"
#include "rt_nonfinite.h"
#include "emlrt.h"
#include "mwmathutil.h"
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_char_T_1x12
#define struct_emxArray_char_T_1x12
struct emxArray_char_T_1x12 {
  char_T data[12];
  int32_T size[2];
};
#endif /* struct_emxArray_char_T_1x12 */
#ifndef typedef_emxArray_char_T_1x12
#define typedef_emxArray_char_T_1x12
typedef struct emxArray_char_T_1x12 emxArray_char_T_1x12;
#endif /* typedef_emxArray_char_T_1x12 */

/* Variable Definitions */
static emlrtRSInfo j_emlrtRSI = {
    102,       /* lineNo */
    "num2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo k_emlrtRSI = {
    92,        /* lineNo */
    "num2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo l_emlrtRSI = {
    90,        /* lineNo */
    "num2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo m_emlrtRSI = {
    88,        /* lineNo */
    "num2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo n_emlrtRSI = {
    395,                /* lineNo */
    "int2strForVector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo o_emlrtRSI = {
    388,                /* lineNo */
    "int2strForVector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo p_emlrtRSI = {
    385,                /* lineNo */
    "int2strForVector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo q_emlrtRSI = {
    94,        /* lineNo */
    "int2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\int2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo r_emlrtRSI = {
    25,                 /* lineNo */
    "printNumToBuffer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\printNumToBuffer.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    23,        /* lineNo */
    "strtrim", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strtrim.m" /* pathName
                                                                           */
};

static emlrtRSInfo t_emlrtRSI = {
    24,        /* lineNo */
    "strtrim", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strtrim.m" /* pathName
                                                                           */
};

static emlrtRSInfo u_emlrtRSI = {
    32,         /* lineNo */
    "firstcol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strtrim.m" /* pathName
                                                                           */
};

static emlrtRSInfo v_emlrtRSI = {
    51,          /* lineNo */
    "allwspace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strtrim.m" /* pathName
                                                                           */
};

static emlrtRSInfo
    w_emlrtRSI =
        {
            15,          /* lineNo */
            "isstrprop", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\isstrpro"
            "p.m" /* pathName */
};

static emlrtRSInfo
    x_emlrtRSI =
        {
            23,                         /* lineNo */
            "apply_property_predicate", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\isstrpro"
            "p.m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    41,        /* lineNo */
    "lastcol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strtrim.m" /* pathName
                                                                           */
};

static emlrtRSInfo ab_emlrtRSI =
    {
        10,            /* lineNo */
        "makeCString", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\makeCString.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    220,                      /* lineNo */
    "handleNumericPrecision", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo db_emlrtRSI = {
    207,                      /* lineNo */
    "handleNumericPrecision", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo eb_emlrtRSI = {
    195,                      /* lineNo */
    "handleNumericPrecision", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo fb_emlrtRSI = {
    180,                      /* lineNo */
    "handleNumericPrecision", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo gb_emlrtRSI = {
    292,               /* lineNo */
    "formatGenerator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo hb_emlrtRSI = {
    293,               /* lineNo */
    "formatGenerator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo ib_emlrtRSI = {
    301,               /* lineNo */
    "formatGenerator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo jb_emlrtRSI =
    {
        40,            /* lineNo */
        "fixEncoding", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\fixEncoding.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI =
    {
        46,            /* lineNo */
        "fixEncoding", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\fixEncoding.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    250,                       /* lineNo */
    "generateFormattedString", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo mb_emlrtRSI = {
    248,                       /* lineNo */
    "generateFormattedString", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo nb_emlrtRSI = {
    247,                       /* lineNo */
    "generateFormattedString", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo ob_emlrtRSI = {
    322,             /* lineNo */
    "formatInteger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo pb_emlrtRSI = {
    324,             /* lineNo */
    "formatInteger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo qb_emlrtRSI = {
    326,             /* lineNo */
    "formatInteger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo rb_emlrtRSI = {
    330,             /* lineNo */
    "formatInteger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pathName
                                                                           */
};

static emlrtRSInfo sb_emlrtRSI = {
    19,        /* lineNo */
    "deblank", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\deblank.m" /* pathName
                                                                           */
};

static emlrtRSInfo tb_emlrtRSI = {
    27,              /* lineNo */
    "deblank_ncols", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\deblank.m" /* pathName
                                                                           */
};

static emlrtRSInfo ub_emlrtRSI = {
    37,          /* lineNo */
    "allwspace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\deblank.m" /* pathName
                                                                           */
};

static emlrtMCInfo emlrtMCI = {
    26,                 /* lineNo */
    9,                  /* colNo */
    "printNumToBuffer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\printNumToBuffer.m" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    350,              /* lineNo */
    5,                /* colNo */
    "formatToString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pName
                                                                           */
};

static emlrtRTEInfo i_emlrtRTEI = {
    351,              /* lineNo */
    1,                /* colNo */
    "formatToString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pName
                                                                           */
};

static emlrtRTEInfo j_emlrtRTEI =
    {
        41,            /* lineNo */
        9,             /* colNo */
        "fixEncoding", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\fixEncoding.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    15,                      /* lineNo */
    9,                       /* colNo */
    "assertSupportedString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertSupportedString.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    254,                       /* lineNo */
    9,                         /* colNo */
    "generateFormattedString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\num2str.m" /* pName
                                                                           */
};

static emlrtRSInfo vb_emlrtRSI = {
    26,                 /* lineNo */
    "printNumToBuffer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\printNumToBuffer.m" /* pathName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y_data[], int32_T y_size[2]);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               char_T ret_data[], int32_T ret_size[2]);

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_feval_,
                             const char_T *identifier, char_T y_data[],
                             int32_T y_size[2]);

static const mxArray *feval(const emlrtStack *sp, const mxArray *m,
                            const mxArray *m1, const mxArray *m2,
                            emlrtMCInfo *location);

static void generateFormattedString(const emlrtStack *sp, real_T x,
                                    const char_T cfmt_data[],
                                    const int32_T cfmt_size[2], char_T s[24]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y_data[], int32_T y_size[2])
{
  c_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               char_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {1, 1};
  boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                            (const void *)&dims[0], &bv[0], &ret_size[0]);
  emlrtImportArrayR2015b((emlrtConstCTX)sp, src, &ret_data[0], 1, false);
  emlrtDestroyArray(&src);
}

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_feval_,
                             const char_T *identifier, char_T y_data[],
                             int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_feval_), &thisId, y_data,
                     y_size);
  emlrtDestroyArray(&a__output_of_feval_);
}

static const mxArray *feval(const emlrtStack *sp, const mxArray *m,
                            const mxArray *m1, const mxArray *m2,
                            emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  const mxArray *m3;
  pArrays[0] = m;
  pArrays[1] = m1;
  pArrays[2] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m3, 3, &pArrays[0],
                               "feval", true, location);
}

static void generateFormattedString(const emlrtStack *sp, real_T x,
                                    const char_T cfmt_data[],
                                    const int32_T cfmt_size[2], char_T s[24])
{
  static const char_T destEncoding[6] = "UTF-8";
  static const char_T str[6] = "%I64d";
  static const char_T b_str[3] = "%e";
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  int32_T cStr_size[2];
  int32_T i;
  int32_T outLength;
  char_T b_st[30];
  char_T formattedElement_data[30];
  char_T b_cfmt_data[24];
  char_T st[24];
  char_T c_st[20];
  char_T outBuff[12];
  boolean_T p;
  d_st.prev = sp;
  d_st.tls = sp->tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  for (i = 0; i < 24; i++) {
    s[i] = ' ';
  }
  d_st.site = &nb_emlrtRSI;
  e_st.site = &ab_emlrtRSI;
  fixEncoding(&e_st, outBuff, cStr_size);
  p = (cfmt_size[1] == cStr_size[1]);
  if (p && (cfmt_size[1] != 0) && (cStr_size[1] != 0)) {
    boolean_T exitg1;
    outLength = 0;
    exitg1 = false;
    while ((!exitg1) && (outLength <= cStr_size[1] - 1)) {
      if (cfmt_data[outLength] != outBuff[outLength]) {
        p = false;
        exitg1 = true;
      } else {
        outLength++;
      }
    }
  }
  if (p) {
    d_st.site = &mb_emlrtRSI;
    if (x >= 9.2233720368547758E+18) {
      e_st.site = &ob_emlrtRSI;
      f_st.site = &ab_emlrtRSI;
      outLength =
          emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0],
                                 0, 3, (char_T *)&b_str[0], &outBuff[0]);
      g_st.site = &bb_emlrtRSI;
      if (outLength == -1) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &b_emlrtRTEI, "Coder:toolbox:EncodingInvalidEncoding",
            "Coder:toolbox:EncodingInvalidEncoding", 0);
      }
      if (outLength == -2) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
            "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (outLength == -3) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
            "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (outLength == -4) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
            "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (outLength == -5) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
            "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (outLength <= -6) {
        emlrtErrorWithMessageIdR2018a(&g_st, &g_emlrtRTEI,
                                      "Coder:toolbox:EncodingUnknown",
                                      "Coder:toolbox:EncodingUnknown", 0);
      }
      if (outLength < 1) {
        outLength = 0;
      }
      if (outLength - 1 >= 0) {
        memcpy(&b_cfmt_data[0], &outBuff[0],
               (uint32_T)outLength * sizeof(char_T));
      }
      e_st.site = &pb_emlrtRSI;
      for (i = 0; i < 20; i++) {
        c_st[i] = ' ';
      }
      outLength = snprintf(&c_st[0], 20, &b_cfmt_data[0], x);
      if (outLength < 0) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &h_emlrtRTEI, "Coder:toolbox:SprintfCallFailed",
            "Coder:toolbox:SprintfCallFailed", 2, 12, outLength);
      }
      if (outLength > 20) {
        emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
      if (outLength - 1 >= 0) {
        memcpy(&formattedElement_data[0], &c_st[0],
               (uint32_T)outLength * sizeof(char_T));
      }
    } else {
      int64_T i1;
      real_T d;
      e_st.site = &qb_emlrtRSI;
      f_st.site = &ab_emlrtRSI;
      outLength =
          emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0],
                                 0, 6, (char_T *)&str[0], &st[0]);
      g_st.site = &bb_emlrtRSI;
      if (outLength == -1) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &b_emlrtRTEI, "Coder:toolbox:EncodingInvalidEncoding",
            "Coder:toolbox:EncodingInvalidEncoding", 0);
      }
      if (outLength == -2) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
            "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (outLength == -3) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
            "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (outLength == -4) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
            "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (outLength == -5) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
            "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (outLength <= -6) {
        emlrtErrorWithMessageIdR2018a(&g_st, &g_emlrtRTEI,
                                      "Coder:toolbox:EncodingUnknown",
                                      "Coder:toolbox:EncodingUnknown", 0);
      }
      if (outLength < 1) {
        outLength = 0;
      }
      if (outLength - 1 >= 0) {
        memcpy(&b_cfmt_data[0], &st[0], (uint32_T)outLength * sizeof(char_T));
      }
      e_st.site = &rb_emlrtRSI;
      for (i = 0; i < 30; i++) {
        b_st[i] = ' ';
      }
      d = muDoubleScalarRound(x);
      if (d < 9.2233720368547758E+18) {
        if (d >= -9.2233720368547758E+18) {
          i1 = (int64_T)d;
        } else {
          i1 = MIN_int64_T;
        }
      } else {
        i1 = 0LL;
      }
      outLength = snprintf(&b_st[0], 30, &b_cfmt_data[0], i1);
      if (outLength < 0) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &h_emlrtRTEI, "Coder:toolbox:SprintfCallFailed",
            "Coder:toolbox:SprintfCallFailed", 2, 12, outLength);
      }
      if (outLength > 30) {
        emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
      if (outLength - 1 >= 0) {
        memcpy(&formattedElement_data[0], &b_st[0],
               (uint32_T)outLength * sizeof(char_T));
      }
    }
  } else {
    d_st.site = &lb_emlrtRSI;
    for (i = 0; i < 24; i++) {
      st[i] = ' ';
    }
    outLength = snprintf(&st[0], 24, &cfmt_data[0], x);
    if (outLength < 0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &h_emlrtRTEI, "Coder:toolbox:SprintfCallFailed",
          "Coder:toolbox:SprintfCallFailed", 2, 12, outLength);
    }
    if (outLength > 24) {
      emlrtErrorWithMessageIdR2018a(&d_st, &i_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (outLength - 1 >= 0) {
      memcpy(&formattedElement_data[0], &st[0],
             (uint32_T)outLength * sizeof(char_T));
    }
  }
  if (outLength > 24) {
    emlrtErrorWithMessageIdR2018a(sp, &l_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (outLength - 1 >= 0) {
    memcpy(&s[0], &formattedElement_data[0],
           (uint32_T)outLength * sizeof(char_T));
  }
}

void num2str(const emlrtStack *sp, real_T x, char_T s_data[], int32_T s_size[2])
{
  static const int32_T iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 4};
  static const char_T str[13] = "%%%.0f.%.0fg";
  static const char_T u[7] = {'s', 'p', 'r', 'i', 'n', 't', 'f'};
  static const char_T destEncoding[6] = "UTF-8";
  static const char_T mlfmt[4] = {'%', '.', '0', 'f'};
  static const boolean_T bv[128] = {
      false, false, false, false, false, false, false, false, false, true,
      true,  true,  true,  true,  false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, true,  true,
      true,  true,  true,  false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emxArray_char_T_1x12 wb_emlrtRSI;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  real_T unnamed_idx_0;
  int32_T currInt_size[2];
  int32_T i;
  char_T fmtGen_data[52];
  char_T outBuff[52];
  char_T outBuff_data[36];
  char_T sArr[25];
  char_T formattedArray[24];
  char_T str_data[9];
  char_T st[8];
  b_st.prev = sp;
  b_st.tls = sp->tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  unnamed_idx_0 = x;
  if (x == 0.0) {
    unnamed_idx_0 = 0.0;
  }
  if (unnamed_idx_0 == unnamed_idx_0) {
    int32_T b_j1;
    int32_T j2;
    char_T b_s_data[3];
    char_T currInt_data[3];
    uint8_T b_u;
    boolean_T exitg1;
    boolean_T p;
    b_st.site = &m_emlrtRSI;
    c_st.site = &p_emlrtRSI;
    b_s_data[0] = ' ';
    b_s_data[1] = ' ';
    b_s_data[2] = ' ';
    c_st.site = &o_emlrtRSI;
    if (unnamed_idx_0 == 0.0) {
      currInt_size[1] = 1;
      currInt_data[0] = '0';
    } else {
      d_st.site = &q_emlrtRSI;
      e_st.site = &r_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(&d_st, 7, m, &u[0]);
      emlrtAssign(&y, m);
      b_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(&d_st, 4, m, &mlfmt[0]);
      emlrtAssign(&b_y, m);
      e_st.site = &vb_emlrtRSI;
      emlrt_marshallIn(
          &e_st,
          feval(&e_st, y, b_y, emlrt_marshallOut(unnamed_idx_0), &emlrtMCI),
          "<output of feval>", currInt_data, currInt_size);
    }
    if (currInt_size[1] >= 1) {
      b_s_data[2] = currInt_data[0];
    }
    c_st.site = &n_emlrtRSI;
    d_st.site = &s_emlrtRSI;
    b_j1 = 0;
    exitg1 = false;
    while ((!exitg1) && (b_j1 + 1 <= 3)) {
      e_st.site = &u_emlrtRSI;
      f_st.site = &v_emlrtRSI;
      g_st.site = &w_emlrtRSI;
      h_st.site = &x_emlrtRSI;
      b_u = (uint8_T)b_s_data[b_j1];
      if (b_u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &h_st, &k_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if ((!bv[b_u]) || (b_s_data[b_j1] == '\x00')) {
        p = false;
      } else {
        p = true;
      }
      if (p) {
        b_j1++;
      } else {
        exitg1 = true;
      }
    }
    d_st.site = &t_emlrtRSI;
    j2 = 2;
    exitg1 = false;
    while ((!exitg1) && (j2 + 1 > 0)) {
      e_st.site = &y_emlrtRSI;
      f_st.site = &v_emlrtRSI;
      g_st.site = &w_emlrtRSI;
      h_st.site = &x_emlrtRSI;
      b_u = (uint8_T)b_s_data[j2];
      if (b_u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &h_st, &k_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if ((!bv[b_u]) || (b_s_data[j2] == '\x00')) {
        p = false;
      } else {
        p = true;
      }
      if (p) {
        j2--;
      } else {
        exitg1 = true;
      }
    }
    if (b_j1 + 1 > j2 + 1) {
      b_j1 = 0;
      j2 = -1;
    }
    s_size[0] = 1;
    j2 -= b_j1;
    s_size[1] = j2 + 1;
    for (i = 0; i <= j2; i++) {
      s_data[i] = b_s_data[b_j1 + i];
    }
    b_st.site = &l_emlrtRSI;
    b_st.site = &k_emlrtRSI;
    c_st.site = &ab_emlrtRSI;
    fixEncoding(&c_st, wb_emlrtRSI.data, wb_emlrtRSI.size);
  } else {
    int32_T b_j1;
    int32_T j2;
    uint8_T b_u;
    boolean_T exitg1;
    boolean_T p;
    if (muDoubleScalarMax(0.0, muDoubleScalarAbs(unnamed_idx_0)) == 0.0) {
      b_j1 = 1;
    } else {
      b_j1 = 5;
    }
    b_st.site = &j_emlrtRSI;
    c_st.site = &fb_emlrtRSI;
    d_st.site = &gb_emlrtRSI;
    e_st.site = &ab_emlrtRSI;
    j2 = emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0], 0,
                                13, (char_T *)&str[0], &outBuff[0]);
    f_st.site = &bb_emlrtRSI;
    if (j2 == -1) {
      emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
                                    "Coder:toolbox:EncodingInvalidEncoding",
                                    "Coder:toolbox:EncodingInvalidEncoding", 0);
    }
    if (j2 == -2) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
          "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
    }
    if (j2 == -3) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
          "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12,
          "windows-1252");
    }
    if (j2 == -4) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
          "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
    }
    if (j2 == -5) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
          "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12,
          "windows-1252");
    }
    if (j2 <= -6) {
      emlrtErrorWithMessageIdR2018a(&f_st, &g_emlrtRTEI,
                                    "Coder:toolbox:EncodingUnknown",
                                    "Coder:toolbox:EncodingUnknown", 0);
    }
    if (j2 < 1) {
      j2 = 0;
    }
    if (j2 - 1 >= 0) {
      memcpy(&fmtGen_data[0], &outBuff[0], (uint32_T)j2 * sizeof(char_T));
    }
    d_st.site = &hb_emlrtRSI;
    for (i = 0; i < 8; i++) {
      st[i] = ' ';
    }
    j2 = snprintf(&st[0], 8, &fmtGen_data[0], (real_T)b_j1 + 7.0, (real_T)b_j1);
    if (j2 < 0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &h_emlrtRTEI, "Coder:toolbox:SprintfCallFailed",
          "Coder:toolbox:SprintfCallFailed", 2, 12, j2);
    }
    if (j2 > 8) {
      emlrtErrorWithMessageIdR2018a(&d_st, &i_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    d_st.site = &ib_emlrtRSI;
    e_st.site = &ab_emlrtRSI;
    if (j2 - 1 >= 0) {
      memcpy(&str_data[0], &st[0], (uint32_T)j2 * sizeof(char_T));
    }
    str_data[j2] = '\x00';
    b_j1 = emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0],
                                  0, j2 + 1, &str_data[0], NULL);
    f_st.site = &jb_emlrtRSI;
    if (b_j1 == -1) {
      emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
                                    "Coder:toolbox:EncodingInvalidEncoding",
                                    "Coder:toolbox:EncodingInvalidEncoding", 0);
    }
    if (b_j1 == -2) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
          "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
    }
    if (b_j1 == -3) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
          "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12,
          "windows-1252");
    }
    if (b_j1 == -4) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
          "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
    }
    if (b_j1 == -5) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
          "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12,
          "windows-1252");
    }
    if (b_j1 <= -6) {
      emlrtErrorWithMessageIdR2018a(&f_st, &g_emlrtRTEI,
                                    "Coder:toolbox:EncodingUnknown",
                                    "Coder:toolbox:EncodingUnknown", 0);
    }
    if (b_j1 > ((j2 + 1) << 2)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &j_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    f_st.site = &kb_emlrtRSI;
    if (b_j1 < 0) {
      emlrtNonNegativeCheckR2012b(b_j1, &emlrtDCI, &f_st);
    }
    currInt_size[0] = 1;
    currInt_size[1] = b_j1;
    if (b_j1 != 0) {
      j2 = emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0],
                                  0, j2 + 1, &str_data[0], &outBuff_data[0]);
      f_st.site = &bb_emlrtRSI;
      if (j2 == -1) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &b_emlrtRTEI, "Coder:toolbox:EncodingInvalidEncoding",
            "Coder:toolbox:EncodingInvalidEncoding", 0);
      }
      if (j2 == -2) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
            "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (j2 == -3) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
            "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (j2 == -4) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
            "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
      }
      if (j2 == -5) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
            "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12,
            "windows-1252");
      }
      if (j2 <= -6) {
        emlrtErrorWithMessageIdR2018a(&f_st, &g_emlrtRTEI,
                                      "Coder:toolbox:EncodingUnknown",
                                      "Coder:toolbox:EncodingUnknown", 0);
      }
    }
    c_st.site = &eb_emlrtRSI;
    generateFormattedString(&c_st, unnamed_idx_0, outBuff_data, currInt_size,
                            formattedArray);
    for (i = 0; i < 25; i++) {
      sArr[i] = ' ';
    }
    c_st.site = &db_emlrtRSI;
    d_st.site = &sb_emlrtRSI;
    j2 = 24;
    exitg1 = false;
    while ((!exitg1) && (j2 > 0)) {
      e_st.site = &tb_emlrtRSI;
      b_u = (uint8_T)formattedArray[j2 - 1];
      if (b_u != 0) {
        f_st.site = &ub_emlrtRSI;
        g_st.site = &w_emlrtRSI;
        h_st.site = &x_emlrtRSI;
        if (b_u > 127) {
          emlrtErrorWithMessageIdR2018a(
              &h_st, &k_emlrtRTEI, "Coder:toolbox:unsupportedString",
              "Coder:toolbox:unsupportedString", 2, 12, 127);
        }
        if (!bv[b_u]) {
          p = false;
        } else {
          p = true;
        }
      } else {
        p = true;
      }
      if (p) {
        j2--;
      } else {
        exitg1 = true;
      }
    }
    if (j2 - 1 >= 0) {
      memcpy(&sArr[0], &formattedArray[0], (uint32_T)j2 * sizeof(char_T));
    }
    c_st.site = &cb_emlrtRSI;
    d_st.site = &s_emlrtRSI;
    b_j1 = 0;
    exitg1 = false;
    while ((!exitg1) && (b_j1 + 1 <= 25)) {
      e_st.site = &u_emlrtRSI;
      f_st.site = &v_emlrtRSI;
      g_st.site = &w_emlrtRSI;
      h_st.site = &x_emlrtRSI;
      b_u = (uint8_T)sArr[b_j1];
      if (b_u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &h_st, &k_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if ((!bv[b_u]) || (sArr[b_j1] == '\x00')) {
        p = false;
      } else {
        p = true;
      }
      if (p) {
        b_j1++;
      } else {
        exitg1 = true;
      }
    }
    d_st.site = &t_emlrtRSI;
    j2 = 24;
    exitg1 = false;
    while ((!exitg1) && (j2 + 1 > 0)) {
      e_st.site = &y_emlrtRSI;
      f_st.site = &v_emlrtRSI;
      g_st.site = &w_emlrtRSI;
      h_st.site = &x_emlrtRSI;
      b_u = (uint8_T)sArr[j2];
      if (b_u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &h_st, &k_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if ((!bv[b_u]) || (sArr[j2] == '\x00')) {
        p = false;
      } else {
        p = true;
      }
      if (p) {
        j2--;
      } else {
        exitg1 = true;
      }
    }
    if (b_j1 + 1 > j2 + 1) {
      b_j1 = 0;
      j2 = -1;
    }
    s_size[0] = 1;
    j2 -= b_j1;
    s_size[1] = j2 + 1;
    for (i = 0; i <= j2; i++) {
      s_data[i] = sArr[b_j1 + i];
    }
  }
}

/* End of code generation (num2str.c) */
