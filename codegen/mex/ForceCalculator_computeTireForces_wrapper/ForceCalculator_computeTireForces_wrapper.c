/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * ForceCalculator_computeTireForces_wrapper.c
 *
 * Code generation for function 'ForceCalculator_computeTireForces_wrapper'
 *
 */

/* Include files */
#include "ForceCalculator_computeTireForces_wrapper.h"
#include "ForceCalculator_computeTireForces_wrapper_data.h"
#include "ForceCalculator_computeTireForces_wrapper_mexutil.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_ForceCalculator
#define typedef_ForceCalculator
typedef struct {
  char_T vehicleType[7];
  char_T tireModelFlag[6];
} ForceCalculator;
#endif /* typedef_ForceCalculator */

/* Variable Definitions */
static boolean_T fc_not_empty;

static emlrtRSInfo emlrtRSI = {
    34,                                          /* lineNo */
    "ForceCalculator_computeTireForces_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\ForceCalculator_computeTireForces_wrapp"
    "er.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    27,                                          /* lineNo */
    "ForceCalculator_computeTireForces_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\ForceCalculator_computeTireForces_wrapp"
    "er.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    53,        /* lineNo */
    "warning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\warning.m" /* pathName
                                                                         */
};

static emlrtRSInfo d_emlrtRSI = {
    322,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    340,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    341,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    498,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    499,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    519,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    520,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    521,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo o_emlrtRSI = {
    401,                                 /* lineNo */
    "ForceCalculator/computeTireForces", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    124,                                        /* lineNo */
    "Pacejka96TireModel/calculateLateralForce", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Pacejka96TireModel.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    119,                                        /* lineNo */
    "Pacejka96TireModel/calculateLateralForce", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Pacejka96TireModel.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    114,                                        /* lineNo */
    "Pacejka96TireModel/calculateLateralForce", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Pacejka96TireModel.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    107,                                        /* lineNo */
    "Pacejka96TireModel/calculateLateralForce", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Pacejka96TireModel.m" /* pathName */
};

static emlrtMCInfo emlrtMCI = {
    84,                         /* lineNo */
    21,                         /* colNo */
    "WarningState/callWarning", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pName */
};

static emlrtMCInfo b_emlrtMCI = {
    27,      /* lineNo */
    5,       /* colNo */
    "error", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pName
                                                                       */
};

static emlrtRSInfo t_emlrtRSI = {
    84,                         /* lineNo */
    "WarningState/callWarning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    27,      /* lineNo */
    "error", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pathName
                                                                       */
};

/* Function Declarations */
static void b_error(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location);

static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    const mxArray *m2, emlrtMCInfo *location);

static void feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                  emlrtMCInfo *location);

/* Function Definitions */
static void b_error(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "error",
                        true, location);
}

static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    const mxArray *m2, emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  pArrays[0] = m;
  pArrays[1] = m1;
  pArrays[2] = m2;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 3, &pArrays[0], "feval",
                        true, location);
}

static void feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                  emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "feval",
                        true, location);
}

void ForceCalculator_computeTireForces_wrapper(const emlrtStack *sp,
                                               const real_T loads[4],
                                               const real_T contactAreas[4],
                                               real_T u, real_T v, real_T r,
                                               real_T *F_y_total, real_T *M_z)
{
  static ForceCalculator fc;
  static const int32_T iv1[2] = {1, 22};
  static const int32_T iv2[2] = {1, 7};
  static const int32_T iv3[2] = {1, 7};
  static const int32_T iv4[2] = {1, 61};
  static const int32_T iv5[2] = {1, 61};
  static const int32_T iv6[2] = {1, 7};
  static const int32_T iv7[2] = {1, 42};
  static const char_T b_varargin_1[61] = {
      'K', '_', 'y', ' ', 'i', 's', ' ', 'i', 'n', 'v', 'a', 'l', 'i',
      'd', ' ', '(', 'N', 'a', 'N', ' ', 'o', 'r', ' ', 'I', 'n', 'f',
      ')', ' ', 'f', 'o', 'r', ' ', 't', 'i', 'r', 'e', ' ', '%', 'd',
      ',', ' ', 's', 'e', 't', 't', 'i', 'n', 'g', ' ', 'F', '_', 'y',
      ' ', 't', 'o', ' ', 'z', 'e', 'r', 'o', '.'};
  static const char_T c_varargin_1[61] = {
      'E', '_', 'y', ' ', 'i', 's', ' ', 'i', 'n', 'v', 'a', 'l', 'i',
      'd', ' ', '(', 'N', 'a', 'N', ' ', 'o', 'r', ' ', 'I', 'n', 'f',
      ')', ' ', 'f', 'o', 'r', ' ', 't', 'i', 'r', 'e', ' ', '%', 'd',
      ',', ' ', 's', 'e', 't', 't', 'i', 'n', 'g', ' ', 'F', '_', 'y',
      ' ', 't', 'o', ' ', 'z', 'e', 'r', 'o', '.'};
  static const char_T d_varargin_1[42] = {
      'W', 'i', 'n', 'd', ' ', 'v', 'e', 'c', 't', 'o', 'r', ' ', 'n', 'o',
      't', ' ', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'd', '.', ' ', 'D', 'e',
      'f', 'a', 'u', 'l', 't', ' ', '[', '0', ';', '0', ';', '0', ']', '.'};
  static const char_T varargin_1[22] = {'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ',
                                        't', 'i', 'r', 'e', ' ', 'i', 'n', 'd',
                                        'e', 'x', ':', ' ', '%', 'd'};
  static const char_T b_b[7] = {'t', 'r', 'a', 'c', 't', 'o', 'r'};
  static const char_T b_u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  static const char_T b[6] = {'s', 'i', 'm', 'p', 'l', 'e'};
  __m128d c_r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  real_T Fy[4];
  real_T alpha[4];
  real_T b_r;
  int32_T tmp_size[2];
  int32_T i;
  int32_T ret;
  (void)contactAreas;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  /*  ForceCalculator_computeTireForces_wrapper - wrapper to call
   * ForceCalculator.computeTireForces for MEX */
  /*  Usage: */
  /*    [F_y_total, M_z] = ForceCalculator_computeTireForces_wrapper(loads,
   * contactAreas, u, v, r) */
  /*  Inputs: */
  /*    loads         - Nx1 vector of tire normal loads */
  /*    contactAreas  - Nx1 vector of contact patch areas */
  /*    u, v, r       - scalar longitudinal, lateral speed and yaw rate */
  if (!fc_not_empty) {
    /*  Basic initialization of ForceCalculator with fixed-size data for codegen
     */
    /*  Constructor for BrakeSystem */
    /*  */
    /*    obj = BrakeSystem(maxBrakingForce, brakeEfficiency, brakeType,
     * brakeResponseRate, brakeBias) */
    /*  */
    /*    Inputs: */
    /*        maxBrakingForce   - Maximum total braking force (N) */
    /*        brakeEfficiency   - Efficiency (0 to 1) */
    /*        brakeType         - Type of brakes ('Disk', 'Drum', 'Air Disk',
     * 'Air Drum') */
    /*        brakeResponseRate - Maximum rate of change of braking force (N/s)
     */
    /*        brakeBias         - Brake bias percentage (Front/Rear %) */
    /*  Default brake bias (50% Front / 50% Rear) */
    /*  Default brake response rate (N/s) */
    /*  Default brake type */
    /*  Default to 100% efficiency */
    /*  Default max braking force (N) */
    /*  Validate brake bias */
    /*  Initialize brake type specific properties */
    st.site = &b_emlrtRSI;
    /*         %% Constructor */
    /*  Constructor for ForceCalculator */
    /*  */
    /*  Preserves all original parameters, plus optional 'wheelSpeeds,
     * wheelRadius, wheelInertia' */
    /*  if given via varargin. */
    /*  Validate vehicleType using codegen friendly syntax */
    for (i = 0; i < 7; i++) {
      fc.vehicleType[i] = b_b[i];
    }
    /*  Assign main properties */
    /*  [u; v; w] */
    /*  [x, y, z, load(N), contactArea(m²)] */
    /*  By default, same for trailer */
    /*  [p; q; r] */
    /*  m/s² */
    /*  Initialize calculated forces struct with zero values */
    /*  initial orientation */
    /*  example */
    /*  --- Tire Model Selection --- */
    for (i = 0; i < 6; i++) {
      fc.tireModelFlag[i] = b[i];
    }
    /*  --- Trailer Yaw Dynamics and Props --- */
    /*  Initialize tire parameters */
    /*         %% initializeTireParameters */
    /*  Initialize friction and rolling-resistance coefficients */
    /*  Initialize flat tire indices */
    /*  --- Wind Vector --- */
    b_st.site = &d_emlrtRSI;
    c_st.site = &c_emlrtRSI;
    f_y = NULL;
    m = emlrtCreateCharArray(2, &iv6[0]);
    emlrtInitCharArrayR2013a(&c_st, 7, m, &b_u[0]);
    emlrtAssign(&f_y, m);
    g_y = NULL;
    m = emlrtCreateCharArray(2, &iv7[0]);
    emlrtInitCharArrayR2013a(&c_st, 42, m, &d_varargin_1[0]);
    emlrtAssign(&g_y, m);
    d_st.site = &t_emlrtRSI;
    feval(&d_st, f_y, g_y, &emlrtMCI);
    /*  --- Braking System --- */
    /*  Calculate axle masses and inertia */
    ret = memcmp(&fc.vehicleType[0], (char_T *)&b_b[0], 7);
    if (ret == 0) {
      b_st.site = &e_emlrtRSI;
      /*         %% calculateAxleMasses */
      /*  Original logic for front/rear mass distribution */
      tmp_size[0] = 1;
      tmp_size[1] = 0;
      c_st.site = &g_emlrtRSI;
      sum(tmp_size);
      tmp_size[0] = 1;
      tmp_size[1] = 1;
      c_st.site = &h_emlrtRSI;
      sum(tmp_size);
      b_st.site = &f_emlrtRSI;
      /*         %% calculateInertia */
      /*  vector from CoG */
      c_st.site = &j_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      c_st.site = &j_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      c_st.site = &k_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      c_st.site = &k_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      c_st.site = &l_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      c_st.site = &l_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      /*  Add trailer inertia to Izz if tractor-trailer */
    }
    /*  --- Filter Setup --- */
    /*  Pre-allocate force buffers and filtered forces for moving average
     * (fixed-size) */
    /*  --- Optional wheel parameters via varargin --- */
    /*  default */
    fc_not_empty = true;
  }
  st.site = &emlrtRSI;
  /*         %% computeTireForces (vectorized lateral forces and yaw moment) */
  b_r = muDoubleScalarAtan2(v - 1.75 * r, u);
  alpha[0] = -b_r;
  alpha[1] = -b_r;
  alpha[2] = -b_r;
  alpha[3] = -b_r;
  ret = memcmp(&fc.tireModelFlag[0], (char_T *)&b[0], 6);
  if (ret == 0) {
    b_r = 10.0 * -b_r;
    b_r = muDoubleScalarSin(
        1.9 * muDoubleScalarAtan(b_r - 0.97 * (b_r - muDoubleScalarAtan(b_r))));
    Fy[0] = 0.8 * loads[0] * b_r;
    Fy[1] = 0.8 * loads[1] * b_r;
    Fy[2] = 0.8 * loads[2] * b_r;
    Fy[3] = 0.8 * loads[3] * b_r;
  } else {
    /*  High-fidelity tire model: compute per-tire forces sequentially */
    for (i = 0; i < 4; i++) {
      real_T d;
      real_T d1;
      b_st.site = &o_emlrtRSI;
      /* /_* */
      /*  * @brief Calculates the lateral force using the Pacejka '96 model for
       * a specific tire. */
      /*  * */
      /*  * This method computes the lateral tire force based on the slip angle
       * and normal load */
      /*  * using the Pacejka '96 tire model equations for the specified tire
       * index. */
      /*  * */
      /*  * @param alpha Slip angle (radians). */
      /*  * @param F_z Normal load on the tire (N). */
      /*  * @param idx Tire index (integer). */
      /*  * */
      /*  * @return F_y The calculated lateral force (N). */
      /*  * */
      /*  * @warning If the normal load is zero or negative, the lateral force
       * is set to zero. */
      /*  * @warning If any of the calculated intermediate variables (C_y, D_y,
       * K_y, E_y, B_y) are */
      /*  * invalid (e.g., zero, NaN, or Inf), the lateral force is set to zero.
       */
      /*  _/ */
      /*  Ensure idx is valid */
      if (i > 0) {
        c_st.site = &s_emlrtRSI;
        y = NULL;
        m = emlrtCreateCharArray(2, &iv1[0]);
        emlrtInitCharArrayR2013a(&c_st, 22, m, &varargin_1[0]);
        emlrtAssign(&y, m);
        d_st.site = &u_emlrtRSI;
        b_error(&d_st, y, emlrt_marshallOut((real_T)i + 1.0), &b_emlrtMCI);
      }
      /*  Extract per-tire parameters */
      c_st.site = &r_emlrtRSI;
      d_st.site = &m_emlrtRSI;
      /*  Handle potential invalid values */
      b_r = loads[i];
      d = 0.0 * b_r;
      if (b_r > 0.0) {
        d1 = rtInf;
      } else if (b_r < 0.0) {
        d1 = rtMinusInf;
      } else {
        d1 = rtNaN;
      }
      if (muDoubleScalarIsNaN(
              b_r * d * muDoubleScalarSin(2.0 * muDoubleScalarAtan(d1)))) {
        c_st.site = &q_emlrtRSI;
        d_st.site = &c_emlrtRSI;
        b_y = NULL;
        m = emlrtCreateCharArray(2, &iv2[0]);
        emlrtInitCharArrayR2013a(&d_st, 7, m, &b_u[0]);
        emlrtAssign(&b_y, m);
        d_y = NULL;
        m = emlrtCreateCharArray(2, &iv4[0]);
        emlrtInitCharArrayR2013a(&d_st, 61, m, &b_varargin_1[0]);
        emlrtAssign(&d_y, m);
        e_st.site = &t_emlrtRSI;
        b_feval(&e_st, b_y, d_y, emlrt_marshallOut(1.0), &emlrtMCI);
      } else if (muDoubleScalarIsNaN(
                     (d + 0.0 * (b_r * b_r)) *
                     (1.0 - 0.0 * muDoubleScalarSign(alpha[i])))) {
        c_st.site = &p_emlrtRSI;
        d_st.site = &c_emlrtRSI;
        c_y = NULL;
        m = emlrtCreateCharArray(2, &iv3[0]);
        emlrtInitCharArrayR2013a(&d_st, 7, m, &b_u[0]);
        emlrtAssign(&c_y, m);
        e_y = NULL;
        m = emlrtCreateCharArray(2, &iv5[0]);
        emlrtInitCharArrayR2013a(&d_st, 61, m, &c_varargin_1[0]);
        emlrtAssign(&e_y, m);
        e_st.site = &t_emlrtRSI;
        b_feval(&e_st, c_y, e_y, emlrt_marshallOut(1.0), &emlrtMCI);
      } else {
        /* warning('B_y is invalid (NaN or Inf) for tire %d, setting F_y to
         * zero.', idx); */
      }
      Fy[i] = 0.0;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    }
  }
  *F_y_total = sumColumnB(Fy);
  c_r = _mm_loadu_pd(&Fy[0]);
  r1 = _mm_set1_pd(0.0);
  _mm_storeu_pd(&Fy[0], _mm_mul_pd(r1, c_r));
  c_r = _mm_loadu_pd(&Fy[2]);
  _mm_storeu_pd(&Fy[2], _mm_mul_pd(r1, c_r));
  *M_z = sumColumnB(Fy);
}

void d_ForceCalculator_computeTireFo(void)
{
  fc_not_empty = false;
}

/* End of code generation (ForceCalculator_computeTireForces_wrapper.c) */
