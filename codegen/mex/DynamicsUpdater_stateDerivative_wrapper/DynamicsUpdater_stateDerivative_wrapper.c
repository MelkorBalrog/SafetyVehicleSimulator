/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * DynamicsUpdater_stateDerivative_wrapper.c
 *
 * Code generation for function 'DynamicsUpdater_stateDerivative_wrapper'
 *
 */

/* Include files */
#include "DynamicsUpdater_stateDerivative_wrapper.h"
#include "DynamicsUpdater_stateDerivative_wrapper_internal_types.h"
#include "ForceCalculator.h"
#include "KinematicsCalculator.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "mwmathutil.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_Clutch
#define typedef_Clutch
typedef struct {
  real_T maxTorque;
  real_T engagementPercentage;
} Clutch;
#endif /* typedef_Clutch */

#ifndef typedef_Transmission
#define typedef_Transmission
typedef struct {
  Clutch *clutch;
} Transmission;
#endif /* typedef_Transmission */

#ifndef typedef_DynamicsUpdater
#define typedef_DynamicsUpdater
typedef struct {
  real_T mass;
  real_T h_CoG;
  ForceCalculator forceCalculator;
  char_T vehicleType[7];
  real_T K_roll;
  real_T C_roll;
} DynamicsUpdater;
#endif /* typedef_DynamicsUpdater */

/* Variable Definitions */
static boolean_T dyn_not_empty;

static emlrtRSInfo emlrtRSI = {
    40,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    49,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    52,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    54,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    33,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    39,                                        /* lineNo */
    "DynamicsUpdater_stateDerivative_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\DynamicsUpdater_stateDerivative_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    53,        /* lineNo */
    "warning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\warning.m" /* pathName
                                                                         */
};

static emlrtRSInfo h_emlrtRSI = {
    321,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    339,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    340,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    497,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    498,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    518,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    519,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    520,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    19,              /* lineNo */
    "Clutch/Clutch", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Clutch.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    45,                          /* lineNo */
    "Transmission/Transmission", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Transmission.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    104,                               /* lineNo */
    "DynamicsUpdater/DynamicsUpdater", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    150,                               /* lineNo */
    "DynamicsUpdater/DynamicsUpdater", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    401,                               /* lineNo */
    "DynamicsUpdater/stateDerivative", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtRSInfo jb_emlrtRSI = {
    405,                               /* lineNo */
    "DynamicsUpdater/stateDerivative", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtMCInfo emlrtMCI = {
    84,                         /* lineNo */
    21,                         /* colNo */
    "WarningState/callWarning", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pName */
};

static emlrtMCInfo f_emlrtMCI = {
    27,      /* lineNo */
    5,       /* colNo */
    "error", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pName
                                                                       */
};

static emlrtRSInfo nb_emlrtRSI = {
    84,                         /* lineNo */
    "WarningState/callWarning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\eml\\+coder\\+internal\\WarningState."
    "m" /* pathName */
};

static emlrtRSInfo ob_emlrtRSI = {
    27,      /* lineNo */
    "error", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" /* pathName
                                                                       */
};

/* Function Declarations */
static void b_error(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location);

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

static void feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                  emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "feval",
                        true, location);
}

void DynamicsUpdater_stateDerivative_wrapper(const emlrtStack *sp,
                                             const real_T stateVec[8],
                                             real_T dydt[8],
                                             real_T accelerations[2])
{
  static Clutch gobj_1;
  static DynamicsUpdater dyn;
  static Transmission gobj_2;
  static const int32_T iv2[5] = {0, 0, 0, 98100, 1};
  static const int32_T iv[2] = {1, 86};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv3[2] = {1, 7};
  static const int32_T iv4[2] = {1, 42};
  static const int32_T iv5[2] = {1, 86};
  static const int32_T iv6[2] = {1, 7};
  static const char_T varargin_1[86] = {
      'U', 'n',  's',  'u', 'p', 'p', 'o',  'r', 't',  'e',  'd', ' ',  'v',
      'e', 'h',  'i',  'c', 'l', 'e', ' ',  't', 'y',  'p',  'e', ':',  ' ',
      '%', 's',  '.',  ' ', 'U', 's', 'e',  ' ', '\'', 't',  'r', 'a',  'c',
      't', 'o',  'r',  '-', 't', 'r', 'a',  'i', 'l',  'e',  'r', '\'', ',',
      ' ', '\'', 't',  'r', 'a', 'c', 't',  'o', 'r',  '\'', ',', ' ',  'o',
      'r', ' ',  '\'', 'P', 'a', 's', 's',  'e', 'n',  'g',  'e', 'r',  'V',
      'e', 'h',  'i',  'c', 'l', 'e', '\'', '.'};
  static const char_T b_varargin_1[42] = {
      'W', 'i', 'n', 'd', ' ', 'v', 'e', 'c', 't', 'o', 'r', ' ', 'n', 'o',
      't', ' ', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'd', '.', ' ', 'D', 'e',
      'f', 'a', 'u', 'l', 't', ' ', '[', '0', ';', '0', ';', '0', ']', '.'};
  static const char_T b[7] = {'t', 'r', 'a', 'c', 't', 'o', 'r'};
  static const char_T b_u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  ForceCalculator fc;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T absxk;
  real_T b_dydt;
  real_T dL_z_dt;
  real_T dp_x_dt;
  real_T dp_y_dt;
  real_T dpdt;
  real_T dphidt;
  real_T dthetadt;
  real_T dxdt;
  real_T expl_temp;
  real_T r;
  real_T scale;
  real_T t;
  real_T u;
  real_T v;
  int32_T tmp_size[2];
  int32_T i;
  int32_T ret;
  char_T a[7];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  /*  DynamicsUpdater_stateDerivative_wrapper - wrapper to call
   * DynamicsUpdater.stateDerivative for MEX */
  /*  Usage: */
  /*    [dydt, accelerations] =
   * DynamicsUpdater_stateDerivative_wrapper(stateVec) */
  /*  Inputs: */
  /*    stateVec - vector [x; y; theta; p_x; p_y; L_z; phi; p] */
  if (!dyn_not_empty) {
    /*  TODO: Initialize DynamicsUpdater with actual instances and initial state
     */
    /*  Placeholder initialState struct */
    /*  Placeholder class instances and parameters */
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
    st.site = &e_emlrtRSI;
    fc.inertia[0] = 0.0;
    fc.inertia[1] = 0.0;
    fc.inertia[2] = 0.0;
    /*         %% Constructor */
    /*  Constructor for ForceCalculator */
    /*  */
    /*  Preserves all original parameters, plus optional 'wheelSpeeds,
     * wheelRadius, wheelInertia' */
    /*  if given via varargin. */
    /*  Validate vehicleType using codegen friendly syntax */
    for (i = 0; i < 7; i++) {
      fc.vehicleType[i] = b[i];
    }
    /*  Assign main properties */
    fc.vehicleMass = 10000.0;
    fc.velocity[0] = 0.0;
    fc.velocity[1] = 0.0;
    fc.velocity[2] = 0.0;
    /*  [u; v; w] */
    for (i = 0; i < 5; i++) {
      fc.loadDistribution[i] = iv2[i];
    }
    /*  [x, y, z, load(N), contactArea(m²)] */
    fc.h_CoG = 1.0;
    /*  By default, same for trailer */
    /*  [p; q; r] */
    fc.trackWidth = 2.0;
    fc.gravity = 9.81;
    /*  m/s² */
    /*  Initialize calculated forces struct with zero values */
    fc.calculatedForces.F_drag_global = 0.0;
    fc.calculatedForces.F_side_global = 0.0;
    fc.calculatedForces.momentZ_wind = 0.0;
    fc.calculatedForces.M_z = 0.0;
    fc.calculatedForces.momentZ = 0.0;
    fc.calculatedForces.F_y_total = 0.0;
    fc.calculatedForces.momentRoll = 0.0;
    fc.calculatedForces.F_y_trailer = 0.0;
    fc.calculatedForces.momentZ_trailer = 0.0;
    fc.calculatedForces.yawMoment_trailer = 0.0;
    fc.calculatedForces.momentRoll_trailer = 0.0;
    fc.calculatedForces.trailerPsi = 0.0;
    fc.calculatedForces.trailerOmega = 0.0;
    fc.calculatedForces.rolloverRiskIndex = 0.0;
    fc.calculatedForces.totalForce[0] = 0.0;
    fc.calculatedForces.totalForce[1] = 0.0;
    fc.calculatedForces.totalForce[2] = 0.0;
    fc.orientation = 0.0;
    /*  initial orientation */
    /*  example */
    /*  --- Tire Model Selection --- */
    /*  --- Trailer Yaw Dynamics and Props --- */
    /*  Initialize tire parameters */
    /*         %% initializeTireParameters */
    fc.B_tires = 10.0;
    fc.C_tires = 1.9;
    fc.D_tires = 1.0;
    fc.E_tires = 0.97;
    /*  Initialize friction and rolling-resistance coefficients */
    /*  Initialize flat tire indices */
    /*  --- Wind Vector --- */
    b_st.site = &h_emlrtRSI;
    c_st.site = &g_emlrtRSI;
    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&c_st, 7, m, &b_u[0]);
    emlrtAssign(&c_y, m);
    d_y = NULL;
    m = emlrtCreateCharArray(2, &iv4[0]);
    emlrtInitCharArrayR2013a(&c_st, 42, m, &b_varargin_1[0]);
    emlrtAssign(&d_y, m);
    d_st.site = &nb_emlrtRSI;
    feval(&d_st, c_y, d_y, &emlrtMCI);
    /*  --- Braking System --- */
    /*  Calculate axle masses and inertia */
    ret = memcmp(&fc.vehicleType[0], (char_T *)&b[0], 7);
    if (ret == 0) {
      b_st.site = &i_emlrtRSI;
      /*         %% calculateAxleMasses */
      /*  Original logic for front/rear mass distribution */
      tmp_size[0] = 1;
      tmp_size[1] = 0;
      c_st.site = &k_emlrtRSI;
      sum(tmp_size);
      tmp_size[0] = 1;
      tmp_size[1] = 1;
      c_st.site = &l_emlrtRSI;
      sum(tmp_size);
      b_st.site = &j_emlrtRSI;
      /*         %% calculateInertia */
      /*  vector from CoG */
      c_st.site = &n_emlrtRSI;
      c_st.site = &n_emlrtRSI;
      c_st.site = &o_emlrtRSI;
      c_st.site = &o_emlrtRSI;
      c_st.site = &p_emlrtRSI;
      c_st.site = &p_emlrtRSI;
      b_dydt = fc.loadDistribution[2] * fc.loadDistribution[2];
      dthetadt = fc.loadDistribution[1] * fc.loadDistribution[1];
      fc.inertia[0] = 10000.0 * (dthetadt + b_dydt);
      scale = fc.loadDistribution[0] * fc.loadDistribution[0];
      fc.inertia[1] = 10000.0 * (scale + b_dydt);
      fc.inertia[2] = 10000.0 * (scale + dthetadt);
      /*  Add trailer inertia to Izz if tractor-trailer */
    }
    /*  --- Filter Setup --- */
    fc.filterWindowSize = 5.0;
    /*  Pre-allocate force buffers and filtered forces for moving average
     * (fixed-size) */
    fc.forceBuffers.hitchLateralForce.sum = 0.0;
    fc.forceBuffers.hitchLateralForce.index = 0.0;
    fc.forceBuffers.hitchLateralForce.count = 0.0;
    fc.forceBuffers.F_drag_global.sum = 0.0;
    fc.forceBuffers.F_drag_global.index = 0.0;
    fc.forceBuffers.F_drag_global.count = 0.0;
    fc.forceBuffers.F_side_global.sum = 0.0;
    fc.forceBuffers.F_side_global.index = 0.0;
    fc.forceBuffers.F_side_global.count = 0.0;
    fc.forceBuffers.momentZ_wind.sum = 0.0;
    fc.forceBuffers.momentZ_wind.index = 0.0;
    fc.forceBuffers.momentZ_wind.count = 0.0;
    fc.forceBuffers.M_z.sum = 0.0;
    fc.forceBuffers.M_z.index = 0.0;
    fc.forceBuffers.M_z.count = 0.0;
    fc.forceBuffers.momentZ.sum = 0.0;
    fc.forceBuffers.momentZ.index = 0.0;
    fc.forceBuffers.momentZ.count = 0.0;
    fc.forceBuffers.F_y_total.sum = 0.0;
    fc.forceBuffers.F_y_total.index = 0.0;
    fc.forceBuffers.F_y_total.count = 0.0;
    fc.forceBuffers.momentRoll.sum = 0.0;
    fc.forceBuffers.momentRoll.index = 0.0;
    fc.forceBuffers.momentRoll.count = 0.0;
    fc.forceBuffers.F_y_trailer.sum = 0.0;
    fc.forceBuffers.F_y_trailer.index = 0.0;
    fc.forceBuffers.F_y_trailer.count = 0.0;
    fc.forceBuffers.momentZ_trailer.sum = 0.0;
    fc.forceBuffers.momentZ_trailer.index = 0.0;
    fc.forceBuffers.momentZ_trailer.count = 0.0;
    fc.forceBuffers.yawMoment_trailer.sum = 0.0;
    fc.forceBuffers.yawMoment_trailer.index = 0.0;
    fc.forceBuffers.yawMoment_trailer.count = 0.0;
    fc.forceBuffers.momentRoll_trailer.sum = 0.0;
    fc.forceBuffers.momentRoll_trailer.index = 0.0;
    fc.forceBuffers.momentRoll_trailer.count = 0.0;
    fc.forceBuffers.trailerPsi.sum = 0.0;
    fc.forceBuffers.trailerPsi.index = 0.0;
    fc.forceBuffers.trailerPsi.count = 0.0;
    fc.forceBuffers.trailerOmega.sum = 0.0;
    fc.forceBuffers.trailerOmega.index = 0.0;
    fc.forceBuffers.trailerOmega.count = 0.0;
    fc.forceBuffers.rolloverRiskIndex.sum = 0.0;
    fc.forceBuffers.rolloverRiskIndex.index = 0.0;
    fc.forceBuffers.rolloverRiskIndex.count = 0.0;
    fc.forceBuffers.jackknifeRiskIndex.sum = 0.0;
    fc.forceBuffers.jackknifeRiskIndex.index = 0.0;
    fc.forceBuffers.jackknifeRiskIndex.count = 0.0;
    for (i = 0; i < 5; i++) {
      fc.forceBuffers.hitchLateralForce.data[i] = 0.0;
      fc.forceBuffers.F_drag_global.data[i] = 0.0;
      fc.forceBuffers.F_side_global.data[i] = 0.0;
      fc.forceBuffers.momentZ_wind.data[i] = 0.0;
      fc.forceBuffers.M_z.data[i] = 0.0;
      fc.forceBuffers.momentZ.data[i] = 0.0;
      fc.forceBuffers.F_y_total.data[i] = 0.0;
      fc.forceBuffers.momentRoll.data[i] = 0.0;
      fc.forceBuffers.F_y_trailer.data[i] = 0.0;
      fc.forceBuffers.momentZ_trailer.data[i] = 0.0;
      fc.forceBuffers.yawMoment_trailer.data[i] = 0.0;
      fc.forceBuffers.momentRoll_trailer.data[i] = 0.0;
      fc.forceBuffers.trailerPsi.data[i] = 0.0;
      fc.forceBuffers.trailerOmega.data[i] = 0.0;
      fc.forceBuffers.rolloverRiskIndex.data[i] = 0.0;
      fc.forceBuffers.jackknifeRiskIndex.data[i] = 0.0;
    }
    /*  --- Optional wheel parameters via varargin --- */
    /*  default */
    st.site = &f_emlrtRSI;
    c_KinematicsCalculator_Kinemati(&st, &b_dydt, &dthetadt, &scale, &absxk,
                                    &expl_temp, &t, &u, &v);
    st.site = &emlrtRSI;
    b_st.site = &eb_emlrtRSI;
    gobj_1.maxTorque = 500.0;
    gobj_1.engagementPercentage = 0.0;
    /*  Start fully engaged */
    /*  Initially, the clutch is engaged */
    /*  Update the current torque based on engagementPercentage */
    /*  Fully engaged */
    st.site = &b_emlrtRSI;
    /*         %% Transmission constructor */
    /*  Initializes the transmission state and shift parameters. */
    /*  */
    /*  @param maxGear            Highest selectable gear */
    /*  @param gearRatios         Vector of gear ratios */
    /*  @param finalDriveRatio    Final drive ratio */
    /*  @param shiftUpSpeed       Speed threshold to upshift */
    /*  @param shiftDownSpeed     Speed threshold to downshift */
    /*  @param engineBrakeTorque  Engine braking torque */
    /*  @param shiftDelay         Delay time between shifts */
    /*  @param clutch             Clutch object */
    /*  @param filterWindowSize   Averaging window for speed filter */
    b_st.site = &fb_emlrtRSI;
    /*  Start at first gear */
    /*  Initialize to negative infinity */
    gobj_2.clutch = &gobj_1;
    /*  Initialize clutch */
    /*  *** Initialize New Shifting Properties *** */
    /*  *** End of Initialization *** */
    /*  *** Initialize Moving Average Filter Properties *** */
    /*  Initialize history with the first measurements to avoid startup
     * anomalies */
    /*  Assuming initial speed is 0 */
    /*  Assuming initial acceleration is 0 */
    /*  *** End of Moving Average Initialization *** */
    st.site = &c_emlrtRSI;
    /*  Constructor with roll dynamics and transmission initialization */
    /*  Validate required fields in initialState */
    b_st.site = &gb_emlrtRSI;
    /*  Initialize properties */
    dyn.forceCalculator = fc;
    /*  [x; y] */
    /*  Yaw angle (theta) */
    dyn.mass = 10000.0;
    dyn.h_CoG = 1.0;
    /*  m/s */
    for (i = 0; i < 7; i++) {
      dyn.vehicleType[i] = b[i];
    }
    /*  'tractor-trailer', 'tractor', or 'PassengerVehicle' */
    /*  Initialize roll stiffness and damping */
    dyn.K_roll = 5000.0;
    dyn.C_roll = 500.0;
    /*  Initialize Transmission */
    /*  Initialize simulation time */
    /*  Initialize trailer state if vehicleType is 'tractor-trailer' */
    for (i = 0; i < 7; i++) {
      a[i] = dyn.vehicleType[i];
    }
    ret = memcmp(&a[0], (char_T *)&b[0], 7);
    if (ret != 0) {
      b_st.site = &hb_emlrtRSI;
      for (i = 0; i < 7; i++) {
        a[i] = dyn.vehicleType[i];
      }
      e_y = NULL;
      m = emlrtCreateCharArray(2, &iv5[0]);
      emlrtInitCharArrayR2013a(&b_st, 86, m, &varargin_1[0]);
      emlrtAssign(&e_y, m);
      f_y = NULL;
      m = emlrtCreateCharArray(2, &iv6[0]);
      emlrtInitCharArrayR2013a(&b_st, 7, m, &a[0]);
      emlrtAssign(&f_y, m);
      c_st.site = &ob_emlrtRSI;
      b_error(&c_st, e_y, f_y, &f_emlrtMCI);
    } else {
      /*  No trailer state */
    }
    /*  Initialize accelerations */
    /*  Initialize smoothing coefficient */
    /*  Initialize momentum properties */
    /*  [p_x; p_y] */
    /*  L_z */
    /*  Initialize roll angle and rate */
    /*  Roll angle (phi) */
    /*  Roll rate (p) */
    dyn_not_empty = true;
  }
  st.site = &d_emlrtRSI;
  /*  Compute state derivatives including roll dynamics and momentum */
  /*  Extract state variables */
  /*  Yaw angle */
  /*  Linear momentum in x */
  /*  Linear momentum in y */
  /*  Angular momentum around z-axis */
  /*  Roll angle */
  /*  Roll rate */
  /*  Yaw moment of inertia from ForceCalculator */
  /*  Compute velocities */
  u = stateVec[3] / dyn.mass;
  v = stateVec[4] / dyn.mass;
  r = stateVec[5] / dyn.forceCalculator.inertia[2];
  /*  Compute moment arm based on roll angle and height of CoG */
  /*  Calculate vehicle load */
  /*  Assemble acceleration vector */
  /*  [a_x; a_y; a_z] */
  /*  Create vehicleState struct based on vehicleType */
  for (i = 0; i < 7; i++) {
    a[i] = dyn.vehicleType[i];
  }
  ret = memcmp(&a[0], (char_T *)&b[0], 7);
  if (ret != 0) {
    b_st.site = &ib_emlrtRSI;
    for (i = 0; i < 7; i++) {
      a[i] = dyn.vehicleType[i];
    }
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&b_st, 86, m, &varargin_1[0]);
    emlrtAssign(&y, m);
    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&b_st, 7, m, &a[0]);
    emlrtAssign(&b_y, m);
    c_st.site = &ob_emlrtRSI;
    b_error(&c_st, y, b_y, &f_emlrtMCI);
  }
  /*  Calculate forces based on vehicleState */
  fc = dyn.forceCalculator;
  b_st.site = &jb_emlrtRSI;
  ForceCalculator_calculateForces(&b_st, &fc);
  dyn.forceCalculator = fc;
  /*  Retrieve calculated forces struct */
  /*         %% getCalculatedForces */
  /*  In vehicle frame [F_x; F_y; F_z] */
  /*  Yaw moment (about Z-axis) */
  /*  Rolling moment (about X-axis) */
  /*  Rate of change of momentum */
  dp_x_dt = dyn.forceCalculator.calculatedForces.totalForce[0];
  dp_y_dt = dyn.forceCalculator.calculatedForces.totalForce[1];
  /*  Yaw rate derivative (rate of change of angular momentum) */
  dL_z_dt = dyn.forceCalculator.calculatedForces.momentZ;
  /*  Lateral acceleration (a_y) */
  /*  Roll dynamics using inertia from ForceCalculator */
  /*  Roll moment of inertia from ForceCalculator */
  /*  Gravitational acceleration (m/s²) */
  /*  Restoring moment due to gravity and suspension */
  /*  Roll rate derivative */
  dpdt = ((dyn.forceCalculator.calculatedForces.momentRoll -
           dyn.mass * 9.81 * dyn.h_CoG * muDoubleScalarSin(stateVec[6])) -
          (dyn.K_roll * stateVec[6] + dyn.C_roll * stateVec[7])) /
         dyn.forceCalculator.inertia[0];
  /*  Roll angle derivative */
  dphidt = stateVec[7];
  /*  Position derivatives */
  b_dydt = muDoubleScalarSin(stateVec[2]);
  dthetadt = muDoubleScalarCos(stateVec[2]);
  dxdt = u * dthetadt - v * b_dydt;
  b_dydt = u * b_dydt + v * dthetadt;
  dthetadt = r;
  /*  --- Prevent Position and Orientation Changes When Stationary --- */
  /*  Define small thresholds for forces and moments to consider them negligible
   */
  /*  Newtons */
  /*  Newton-meters */
  /*  Calculate total longitudinal and lateral forces and yaw moment */
  if ((muDoubleScalarAbs(dyn.forceCalculator.calculatedForces.totalForce[0]) <
       0.001) &&
      (muDoubleScalarAbs(dyn.forceCalculator.calculatedForces.totalForce[1]) <
       0.001) &&
      (muDoubleScalarAbs(dyn.forceCalculator.calculatedForces.momentZ) <
       0.001)) {
    scale = 3.3121686421112381E-170;
    absxk = muDoubleScalarAbs(u);
    if (absxk > 3.3121686421112381E-170) {
      expl_temp = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      expl_temp = t * t;
    }
    absxk = muDoubleScalarAbs(v);
    if (absxk > scale) {
      t = scale / absxk;
      expl_temp = expl_temp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      expl_temp += t * t;
    }
    expl_temp = scale * muDoubleScalarSqrt(expl_temp);
    if (expl_temp == 0.0) {
      /*  If all forces and moments are negligible and speed is zero, */
      /*  prevent any state changes by setting derivatives to zero */
      dxdt = 0.0;
      b_dydt = 0.0;
      dthetadt = 0.0;
      dp_x_dt = 0.0;
      dp_y_dt = 0.0;
      dL_z_dt = 0.0;
      dphidt = 0.0;
      dpdt = 0.0;
    }
  }
  /*  Assemble state derivatives */
  dydt[0] = dxdt;
  dydt[1] = b_dydt;
  dydt[2] = dthetadt;
  dydt[3] = dp_x_dt;
  dydt[4] = dp_y_dt;
  dydt[5] = dL_z_dt;
  dydt[6] = dphidt;
  dydt[7] = dpdt;
  /*  Compute accelerations in vehicle frame */
  accelerations[0] =
      dyn.forceCalculator.calculatedForces.totalForce[0] / dyn.mass - r * v;
  accelerations[1] =
      dyn.forceCalculator.calculatedForces.totalForce[1] / dyn.mass + r * u;
}

void d_DynamicsUpdater_stateDerivati(void)
{
  dyn_not_empty = false;
}

/* End of code generation (DynamicsUpdater_stateDerivative_wrapper.c) */
