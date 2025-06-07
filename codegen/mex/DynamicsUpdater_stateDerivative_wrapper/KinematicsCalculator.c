/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * KinematicsCalculator.c
 *
 * Code generation for function 'KinematicsCalculator'
 *
 */

/* Include files */
#include "KinematicsCalculator.h"
#include "rt_nonfinite.h"
#include <stdio.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo s_emlrtRSI = {
    225,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    233,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    242,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    250,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    258,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    267,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    275,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = {
    283,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    291,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    8,          /* lineNo */
    "debugLog", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    38,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtMCInfo b_emlrtMCI = {
    5,          /* lineNo */
    8,          /* colNo */
    "debugLog", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pName */
};

static emlrtMCInfo c_emlrtMCI = {
    5,          /* lineNo */
    5,          /* colNo */
    "debugLog", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pName */
};

static emlrtMCInfo d_emlrtMCI = {
    5,          /* lineNo */
    41,         /* colNo */
    "debugLog", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pName */
};

static emlrtMCInfo e_emlrtMCI = {
    66,        /* lineNo */
    18,        /* colNo */
    "fprintf", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pName
                                                                          */
};

static emlrtRSInfo lb_emlrtRSI = {
    66,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtRSInfo mb_emlrtRSI = {
    5,          /* lineNo */
    "debugLog", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pathName */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u[13]);

static const mxArray *b_feval(const emlrtStack *sp, const mxArray *m,
                              const mxArray *m1, const mxArray *m2,
                              const mxArray *m3, emlrtMCInfo *location);

static const mxArray *c_coder_internal_ifWhileCondExt(const emlrtStack *sp,
                                                      const mxArray *m,
                                                      emlrtMCInfo *location);

static boolean_T
c_emlrt_marshallIn(const emlrtStack *sp,
                   const mxArray *c_a__output_of_coder_internal_i,
                   const char_T *identifier);

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static real_T emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_feval_,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const char_T u[7]);

static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static const mxArray *getappdata(const emlrtStack *sp, const mxArray *m,
                                 const mxArray *m1, emlrtMCInfo *location);

static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static const mxArray *isappdata(const emlrtStack *sp, const mxArray *m,
                                const mxArray *m1, emlrtMCInfo *location);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u[13])
{
  static const int32_T iv[2] = {1, 13};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 13, m, &u[0]);
  emlrtAssign(&y, m);
  return y;
}

static const mxArray *b_feval(const emlrtStack *sp, const mxArray *m,
                              const mxArray *m1, const mxArray *m2,
                              const mxArray *m3, emlrtMCInfo *location)
{
  const mxArray *pArrays[4];
  const mxArray *m4;
  pArrays[0] = m;
  pArrays[1] = m1;
  pArrays[2] = m2;
  pArrays[3] = m3;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m4, 4, &pArrays[0],
                               "feval", true, location);
}

static const mxArray *c_coder_internal_ifWhileCondExt(const emlrtStack *sp,
                                                      const mxArray *m,
                                                      emlrtMCInfo *location)
{
  const mxArray *m1;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m1, 1, &m,
                               "coder.internal.ifWhileCondExtrinsic", true,
                               location);
}

static boolean_T
c_emlrt_marshallIn(const emlrtStack *sp,
                   const mxArray *c_a__output_of_coder_internal_i,
                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  boolean_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(c_a__output_of_coder_internal_i),
                         &thisId);
  emlrtDestroyArray(&c_a__output_of_coder_internal_i);
  return y;
}

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_feval_,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_feval_), &thisId);
  emlrtDestroyArray(&a__output_of_feval_);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const char_T u[7])
{
  static const int32_T iv[2] = {1, 7};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  return y;
}

static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static const mxArray *getappdata(const emlrtStack *sp, const mxArray *m,
                                 const mxArray *m1, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m2;
  pArrays[0] = m;
  pArrays[1] = m1;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m2, 2, &pArrays[0],
                               "getappdata", true, location);
}

static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static const mxArray *isappdata(const emlrtStack *sp, const mxArray *m,
                                const mxArray *m1, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m2;
  pArrays[0] = m;
  pArrays[1] = m1;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m2, 2, &pArrays[0],
                               "isappdata", true, location);
}

real_T c_KinematicsCalculator_Kinemati(
    const emlrtStack *sp, real_T *obj_h_CoG_trailer, real_T *obj_K_roll_tractor,
    real_T *obj_D_roll_tractor, real_T *obj_I_roll_tractor,
    real_T *obj_K_roll_trailer, real_T *obj_D_roll_trailer,
    real_T *obj_I_roll_trailer, real_T *obj_dt)
{
  static const int32_T iv[2] = {1, 35};
  static const int32_T iv1[2] = {1, 35};
  static const int32_T iv2[2] = {1, 37};
  static const int32_T iv3[2] = {1, 39};
  static const int32_T iv4[2] = {1, 35};
  static const int32_T iv5[2] = {1, 37};
  static const int32_T iv6[2] = {1, 60};
  static const int32_T iv7[2] = {1, 35};
  static const int32_T iv8[2] = {1, 25};
  static const char_T g_u[60] = {
      'D', '_', 'r',    'o', 'l',    'l', '_', 't', 'r', 'a', 'i',  'l',
      'e', 'r', ' ',    'n', 'o',    't', ' ', 'p', 'r', 'o', 'v',  'i',
      'd', 'e', 'd',    '.', ' ',    'U', 's', 'i', 'n', 'g', ' ',  'd',
      'e', 'f', 'a',    'u', 'l',    't', ':', ' ', '%', '.', '2',  'f',
      ' ', 'N', '\xb7', 'm', '\xb7', 's', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T d_u[39] = {
      'D', '_', 'r', 'o',    'l', 'l',    '_', 't', 'r', 'a', 'c', 't',  'o',
      'r', ' ', 's', 'e',    't', ' ',    't', 'o', ':', ' ', '%', '.',  '2',
      'f', ' ', 'N', '\xb7', 'm', '\xb7', 's', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T c_u[37] = {
      'K', '_', 'r', 'o',    'l', 'l', '_', 't', 'r', 'a',  'c', 't', 'o',
      'r', ' ', 's', 'e',    't', ' ', 't', 'o', ':', ' ',  '%', '.', '2',
      'f', ' ', 'N', '\xb7', 'm', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T f_u[37] = {
      'K', '_', 'r', 'o',    'l', 'l', '_', 't', 'r', 'a',  'i', 'l', 'e',
      'r', ' ', 's', 'e',    't', ' ', 't', 'o', ':', ' ',  '%', '.', '2',
      'f', ' ', 'N', '\xb7', 'm', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T b_u[35] = {'h', '_', 'C', 'o', 'G', '_', 't',  'r', 'a',
                                 'i', 'l', 'e', 'r', ' ', 's', 'e',  't', ' ',
                                 't', 'o', ':', ' ', '%', '.', '2',  'f', ' ',
                                 'm', 'e', 't', 'e', 'r', 's', '\\', 'n'};
  static const char_T e_u[35] = {
      'I', '_', 'r', 'o', 'l', 'l', '_',    't', 'r',    'a',  'c', 't',
      'o', 'r', ' ', 's', 'e', 't', ' ',    't', 'o',    ':',  ' ', '%',
      '.', '2', 'f', ' ', 'k', 'g', '\xb7', 'm', '\xb2', '\\', 'n'};
  static const char_T h_u[35] = {
      'I', '_', 'r', 'o', 'l', 'l', '_',    't', 'r',    'a',  'i', 'l',
      'e', 'r', ' ', 's', 'e', 't', ' ',    't', 'o',    ':',  ' ', '%',
      '.', '2', 'f', ' ', 'k', 'g', '\xb7', 'm', '\xb2', '\\', 'n'};
  static const char_T u[35] = {'h', '_', 'C', 'o', 'G', '_', 't',  'r', 'a',
                               'c', 't', 'o', 'r', ' ', 's', 'e',  't', ' ',
                               't', 'o', ':', ' ', '%', '.', '2',  'f', ' ',
                               'm', 'e', 't', 'e', 'r', 's', '\\', 'n'};
  static const char_T i_u[25] = {'d', 't', ' ', 's', 'e', 't',  ' ', 't', 'o',
                                 ':', ' ', '%', '.', '4', 'f',  ' ', 's', 'e',
                                 'c', 'o', 'n', 'd', 's', '\\', 'n'};
  static const char_T cv[13] = {'S', 'u', 'p', 'p', 'r', 'e', 's',
                                's', 'D', 'e', 'b', 'u', 'g'};
  static const char_T cv1[7] = {'f', 'p', 'r', 'i', 'n', 't', 'f'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  const mxArray *ab_y;
  const mxArray *b_y;
  const mxArray *bb_y;
  const mxArray *c_y;
  const mxArray *cb_y;
  const mxArray *d_y;
  const mxArray *db_y;
  const mxArray *e_y;
  const mxArray *eb_y;
  const mxArray *f_y;
  const mxArray *fb_y;
  const mxArray *g_y;
  const mxArray *gb_y;
  const mxArray *h_y;
  const mxArray *hb_y;
  const mxArray *i_y;
  const mxArray *ib_y;
  const mxArray *j_y;
  const mxArray *jb_y;
  const mxArray *k_y;
  const mxArray *kb_y;
  const mxArray *l_y;
  const mxArray *lb_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *mb_y;
  const mxArray *n_y;
  const mxArray *nb_y;
  const mxArray *o_y;
  const mxArray *ob_y;
  const mxArray *p_y;
  const mxArray *pb_y;
  const mxArray *q_y;
  const mxArray *qb_y;
  const mxArray *r_y;
  const mxArray *rb_y;
  const mxArray *s_y;
  const mxArray *sb_y;
  const mxArray *t_y;
  const mxArray *tb_y;
  const mxArray *u_y;
  const mxArray *v_y;
  const mxArray *w_y;
  const mxArray *x_y;
  const mxArray *y;
  const mxArray *y_y;
  real_T obj_h_CoG_tractor;
  boolean_T guard1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  /* /_* */
  /*  * @brief Constructor to initialize the KinematicsCalculator. */
  /*  * */
  /*  * Initializes a new instance of the KinematicsCalculator class with the
   * specified center of gravity heights */
  /*  * and roll dynamics parameters for both tractor and trailer. */
  /*  * */
  /*  * @param forceCalc Instance of the ForceCalculator class. */
  /*  * @param h_CoG_tractor (Optional) Height of the tractor's center of
   * gravity (meters). Defaults to 1.5 meters if not provided. */
  /*  * @param h_CoG_trailer (Optional) Height of the trailer's center of
   * gravity (meters). Defaults to 2.0 meters if not provided. */
  /*  * @param K_roll_tractor (Optional) Roll stiffness of the tractor
   * (N·m/rad). Defaults to 200,000 N·m/rad if not provided. */
  /*  * @param D_roll_tractor (Optional) Roll damping coefficient of the tractor
   * (N·m·s/rad). Defaults to 5,000 N·m·s/rad if not provided. */
  /*  * @param I_roll_tractor (Optional) Roll inertia of the tractor (kg·m²).
   * Defaults to 5,000 kg·m² if not provided. */
  /*  * @param K_roll_trailer (Optional) Roll stiffness of the trailer
   * (N·m/rad). Defaults to 150,000 N·m/rad if not provided. */
  /*  * @param D_roll_trailer (Optional) Roll damping coefficient of the trailer
   * (N·m·s/rad). Defaults to 4,000 N·m·s/rad if not provided. */
  /*  * @param I_roll_trailer (Optional) Roll inertia of the trailer (kg·m²).
   * Defaults to 8,000 kg·m² if not provided. */
  /*  * @param dt (Optional) Time step for integration (seconds). Defaults to
   * 0.01 seconds if not provided. */
  /*  * */
  /*  * @throws None */
  /*  _/ */
  /*  Initialize center of gravity heights */
  obj_h_CoG_tractor = 1.0;
  st.site = &s_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, y, b_emlrt_marshallOut(&b_st, cv), &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    b_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&b_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, b_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    c_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&c_y, m);
    d_y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&c_st, 35, m, &u[0]);
    emlrtAssign(&d_y, m);
    f_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&f_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), c_y, d_y,
                             f_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_h_CoG_trailer = 2.0;
  st.site = &t_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  e_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&e_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, e_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    g_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&g_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, g_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    h_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&h_y, m);
    i_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&c_st, 35, m, &b_u[0]);
    emlrtAssign(&i_y, m);
    k_y = NULL;
    m = emlrtCreateDoubleScalar(2.0);
    emlrtAssign(&k_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), h_y, i_y,
                             k_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  /*  Initialize roll dynamics parameters for tractor */
  *obj_K_roll_tractor = 200000.0;
  st.site = &u_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  j_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&j_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, j_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    l_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&l_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, l_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    m_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&m_y, m);
    n_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&c_st, 37, m, &c_u[0]);
    emlrtAssign(&n_y, m);
    p_y = NULL;
    m = emlrtCreateDoubleScalar(200000.0);
    emlrtAssign(&p_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), m_y, n_y,
                             p_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_D_roll_tractor = 5000.0;
  st.site = &v_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  o_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&o_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, o_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    q_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&q_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, q_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    r_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&r_y, m);
    s_y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&c_st, 39, m, &d_u[0]);
    emlrtAssign(&s_y, m);
    u_y = NULL;
    m = emlrtCreateDoubleScalar(5000.0);
    emlrtAssign(&u_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), r_y, s_y,
                             u_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_I_roll_tractor = 5000.0;
  st.site = &w_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  t_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&t_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, t_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    v_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&v_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, v_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    w_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&w_y, m);
    x_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&c_st, 35, m, &e_u[0]);
    emlrtAssign(&x_y, m);
    ab_y = NULL;
    m = emlrtCreateDoubleScalar(5000.0);
    emlrtAssign(&ab_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), w_y, x_y,
                             ab_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  /*  Initialize roll dynamics parameters for trailer */
  *obj_K_roll_trailer = 150000.0;
  st.site = &x_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  y_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&y_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, y_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    bb_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&bb_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, bb_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    cb_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&cb_y, m);
    db_y = NULL;
    m = emlrtCreateCharArray(2, &iv5[0]);
    emlrtInitCharArrayR2013a(&c_st, 37, m, &f_u[0]);
    emlrtAssign(&db_y, m);
    fb_y = NULL;
    m = emlrtCreateDoubleScalar(150000.0);
    emlrtAssign(&fb_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), cb_y, db_y,
                             fb_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_D_roll_trailer = 4000.0;
  st.site = &y_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  eb_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&eb_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, eb_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    gb_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&gb_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, gb_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    hb_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&hb_y, m);
    ib_y = NULL;
    m = emlrtCreateCharArray(2, &iv6[0]);
    emlrtInitCharArrayR2013a(&c_st, 60, m, &g_u[0]);
    emlrtAssign(&ib_y, m);
    kb_y = NULL;
    m = emlrtCreateDoubleScalar(4000.0);
    emlrtAssign(&kb_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), hb_y, ib_y,
                             kb_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_I_roll_trailer = 8000.0;
  st.site = &ab_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  jb_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&jb_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, jb_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    lb_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&lb_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, lb_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    mb_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&mb_y, m);
    nb_y = NULL;
    m = emlrtCreateCharArray(2, &iv7[0]);
    emlrtInitCharArrayR2013a(&c_st, 35, m, &h_u[0]);
    emlrtAssign(&nb_y, m);
    pb_y = NULL;
    m = emlrtCreateDoubleScalar(8000.0);
    emlrtAssign(&pb_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), mb_y, nb_y,
                             pb_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  *obj_dt = 0.01;
  st.site = &bb_emlrtRSI;
  /*  debugLog Conditionally prints debug messages based on global suppression
   * flag. */
  /*  If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise
   * printed via fprintf. */
  /*  Usage: debugLog('Value: %d\n', x); */
  ob_y = NULL;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&ob_y, m);
  guard1 = false;
  b_st.site = &mb_emlrtRSI;
  if (c_emlrt_marshallIn(
          &b_st,
          c_coder_internal_ifWhileCondExt(
              &b_st,
              isappdata(&b_st, ob_y, b_emlrt_marshallOut(&b_st, cv),
                        &b_emlrtMCI),
              &c_emlrtMCI),
          "<output of coder.internal.ifWhileCondExtrinsic>")) {
    qb_y = NULL;
    m = emlrtCreateDoubleScalar(0.0);
    emlrtAssign(&qb_y, m);
    b_st.site = &mb_emlrtRSI;
    if (!c_emlrt_marshallIn(
            &b_st,
            c_coder_internal_ifWhileCondExt(
                &b_st,
                getappdata(&b_st, qb_y, b_emlrt_marshallOut(&b_st, cv),
                           &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_st.site = &cb_emlrtRSI;
    c_st.site = &db_emlrtRSI;
    rb_y = NULL;
    m = emlrtCreateDoubleScalar(1.0);
    emlrtAssign(&rb_y, m);
    sb_y = NULL;
    m = emlrtCreateCharArray(2, &iv8[0]);
    emlrtInitCharArrayR2013a(&c_st, 25, m, &i_u[0]);
    emlrtAssign(&sb_y, m);
    tb_y = NULL;
    m = emlrtCreateDoubleScalar(0.01);
    emlrtAssign(&tb_y, m);
    d_st.site = &lb_emlrtRSI;
    emlrt_marshallIn(&d_st,
                     b_feval(&d_st, emlrt_marshallOut(&d_st, cv1), rb_y, sb_y,
                             tb_y, &e_emlrtMCI),
                     "<output of feval>");
  }
  /*  Assign ForceCalculator instance */
  /*  Initialize lateral accelerations and roll states */
  /*  Initialize original properties */
  /*  Assuming h_CoG refers to tractor's CoG */
  /*  Original lateralAcceleration remains, can be used if needed */
  /*  Add other original initializations as needed */
  return obj_h_CoG_tractor;
}

/* End of code generation (KinematicsCalculator.c) */
