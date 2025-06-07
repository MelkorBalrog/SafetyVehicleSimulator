/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * StabilityChecker_checkStability_wrapper.c
 *
 * Code generation for function 'StabilityChecker_checkStability_wrapper'
 *
 */

/* Include files */
#include "StabilityChecker_checkStability_wrapper.h"
#include "WarningState.h"
#include "error.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T B;
  real_T C;
  real_T D;
  real_T E;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  real_T hitch[3];
  real_T hitchLateralForce;
  real_T F_drag_global;
  real_T F_side_global;
  real_T momentZ_wind;
  real_T traction[3];
  real_T traction_force[3];
  real_T M_z;
  real_T momentZ;
  real_T F_y_total;
  real_T momentRoll;
  real_T totalForce[3];
  real_T F_y_trailer;
  real_T momentZ_trailer;
  real_T F_total_trailer_local[3];
  real_T F_total_trailer_global[3];
  real_T yawMoment_trailer;
  real_T momentRoll_trailer;
  real_T trailerPsi;
  real_T trailerOmega;
  real_T rolloverRiskIndex;
  real_T jackknifeRiskIndex;
  real_T braking[3];
} b_struct_T;
#endif /* typedef_b_struct_T */

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

#ifndef typedef_ForceCalculator
#define typedef_ForceCalculator
typedef struct {
  char_T vehicleType[7];
  real_T frictionCoefficient;
  real_T loadDistribution[5];
  real_T centerOfGravity[3];
  real_T inertia[3];
  real_T gravity;
  b_struct_T calculatedForces;
  struct_T tireModel;
  real_T numTrailerTires;
  char_T tireModelFlag[6];
} ForceCalculator;
#endif /* typedef_ForceCalculator */

#ifndef typedef_DynamicsUpdater
#define typedef_DynamicsUpdater
typedef struct {
  real_T mass;
  ForceCalculator forceCalculator;
  char_T vehicleType[7];
  Transmission *transmission;
} DynamicsUpdater;
#endif /* typedef_DynamicsUpdater */

#ifndef struct_emxArray_real_T_20
#define struct_emxArray_real_T_20
struct emxArray_real_T_20 {
  real_T data[20];
};
#endif /* struct_emxArray_real_T_20 */
#ifndef typedef_emxArray_real_T_20
#define typedef_emxArray_real_T_20
typedef struct emxArray_real_T_20 emxArray_real_T_20;
#endif /* typedef_emxArray_real_T_20 */

#ifndef typedef_StabilityChecker
#define typedef_StabilityChecker
typedef struct {
  emxArray_real_T_20 trailerYawBuffer;
} StabilityChecker;
#endif /* typedef_StabilityChecker */

/* Variable Definitions */
static boolean_T checker_not_empty;

static emlrtRSInfo emlrtRSI = {
    28,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    37,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    47,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    50,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    21,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    27,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    48,                                        /* lineNo */
    "StabilityChecker_checkStability_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\StabilityChecker_checkStability_wrapper"
    ".m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    53,        /* lineNo */
    "warning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\lang\\warning.m" /* pathName
                                                                         */
};

static emlrtRSInfo i_emlrtRSI = {
    321,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    339,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    340,                               /* lineNo */
    "ForceCalculator/ForceCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    497,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    498,                                   /* lineNo */
    "ForceCalculator/calculateAxleMasses", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    518,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    519,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    520,                                /* lineNo */
    "ForceCalculator/calculateInertia", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo s_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRSInfo t_emlrtRSI = {
    225,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    233,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    242,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    250,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    258,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    267,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = {
    275,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    283,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    291,                                         /* lineNo */
    "KinematicsCalculator/KinematicsCalculator", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\KinematicsCalculator.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    8,          /* lineNo */
    "debugLog", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    38,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtRSInfo fb_emlrtRSI = {
    19,              /* lineNo */
    "Clutch/Clutch", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Clutch.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    45,                          /* lineNo */
    "Transmission/Transmission", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Mechanics\\Transmission.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    104,                               /* lineNo */
    "DynamicsUpdater/DynamicsUpdater", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    150,                               /* lineNo */
    "DynamicsUpdater/DynamicsUpdater", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\DynamicsUpdater.m" /* pathName */
};

static emlrtRSInfo jb_emlrtRSI = {
    35,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtRSInfo kb_emlrtRSI = {
    187,                                 /* lineNo */
    "StabilityChecker/StabilityChecker", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\StabilityChecker.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    222,                                 /* lineNo */
    "StabilityChecker/StabilityChecker", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\StabilityChecker.m" /* pathName */
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

static emlrtRSInfo mb_emlrtRSI = {
    66,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtRSInfo nb_emlrtRSI = {
    68,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtRSInfo ob_emlrtRSI = {
    5,          /* lineNo */
    "debugLog", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Simulation\\debugLog.m" /* pathName */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(const real_T u);

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

static const mxArray *c_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u[35]);

static const mxArray *c_feval(const emlrtStack *sp, const mxArray *m,
                              const mxArray *m1, const mxArray *m2,
                              emlrtMCInfo *location);

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static const mxArray *d_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u[13]);

static const mxArray *d_feval(const emlrtStack *sp, const mxArray *m,
                              const mxArray *m1, const mxArray *m2,
                              const mxArray *m3, const mxArray *m4,
                              const mxArray *m5, const mxArray *m6,
                              emlrtMCInfo *location);

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_feval_,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const char_T u[7]);

static boolean_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static const mxArray *getappdata(const emlrtStack *sp, const mxArray *m,
                                 const mxArray *m1, emlrtMCInfo *location);

static const mxArray *isappdata(const emlrtStack *sp, const mxArray *m,
                                const mxArray *m1, emlrtMCInfo *location);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
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

static const mxArray *c_emlrt_marshallOut(const emlrtStack *sp,
                                          const char_T u[35])
{
  static const int32_T iv[2] = {1, 35};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 35, m, &u[0]);
  emlrtAssign(&y, m);
  return y;
}

static const mxArray *c_feval(const emlrtStack *sp, const mxArray *m,
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

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *d_emlrt_marshallOut(const emlrtStack *sp,
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

static const mxArray *d_feval(const emlrtStack *sp, const mxArray *m,
                              const mxArray *m1, const mxArray *m2,
                              const mxArray *m3, const mxArray *m4,
                              const mxArray *m5, const mxArray *m6,
                              emlrtMCInfo *location)
{
  const mxArray *pArrays[7];
  const mxArray *m7;
  pArrays[0] = m;
  pArrays[1] = m1;
  pArrays[2] = m2;
  pArrays[3] = m3;
  pArrays[4] = m4;
  pArrays[5] = m5;
  pArrays[6] = m6;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m7, 7, &pArrays[0],
                               "feval", true, location);
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static boolean_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

void StabilityChecker_checkStability_wrapper(const emlrtStack *sp,
                                             boolean_T *isWiggling,
                                             boolean_T *isRollover,
                                             boolean_T *isSkidding,
                                             boolean_T *isJackknife)
{
  static Clutch gobj_0;
  static DynamicsUpdater gobj_2;
  static StabilityChecker checker;
  static Transmission gobj_1;
  static const int32_T fc_loadDistribution[5] = {0, 0, 0, 98100, 1};
  static const int32_T iv[2] = {1, 37};
  static const int32_T iv1[2] = {1, 39};
  static const int32_T iv2[2] = {1, 37};
  static const int32_T iv3[2] = {1, 60};
  static const int32_T iv4[2] = {1, 25};
  static const int32_T iv5[2] = {1, 63};
  static const int32_T iv6[2] = {1, 79};
  static const char_T g_u[79] = {
      'A', 'u', 't', 'o', '-', 't', 'h', 'r',  'e', 's', 'h', 'o', 'l', 'd',
      ' ', '=', '>', ' ', 'r', 'o', 'l', 'l',  'A', 'n', 'g', 'l', 'e', '=',
      '%', '.', '4', 'f', ',', ' ', 'h', 'i',  't', 'c', 'h', '=', '%', '.',
      '2', 'f', ' ', 'N', ',', ' ', 'm', 'a',  'x', 'A', 'r', 't', 'i', 'c',
      '=', '%', '.', '4', 'f', ',', ' ', 'y',  'a', 'w', 'M', 'o', 'm', 'e',
      'n', 't', '=', '%', '.', '2', 'f', '\\', 'n'};
  static const char_T f_u[63] = {
      'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i',  'n', 'g', ' ',
      'S', 't', 'a', 'b', 'i', 'l', 'i', 't', 'y', 'C',  'h', 'e', 'c',
      'k', 'e', 'r', ' ', '(', 'a', 'u', 't', 'o', '-',  't', 'h', 'r',
      'e', 's', 'h', 'o', 'l', 'd', ' ', 'c', 'a', 'l',  'c', 'u', 'l',
      'a', 't', 'i', 'o', 'n', ')', '.', '.', '.', '\\', 'n'};
  static const char_T d_u[60] = {
      'D', '_', 'r',    'o', 'l',    'l', '_', 't', 'r', 'a', 'i',  'l',
      'e', 'r', ' ',    'n', 'o',    't', ' ', 'p', 'r', 'o', 'v',  'i',
      'd', 'e', 'd',    '.', ' ',    'U', 's', 'i', 'n', 'g', ' ',  'd',
      'e', 'f', 'a',    'u', 'l',    't', ':', ' ', '%', '.', '2',  'f',
      ' ', 'N', '\xb7', 'm', '\xb7', 's', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T b_u[39] = {
      'D', '_', 'r', 'o',    'l', 'l',    '_', 't', 'r', 'a', 'c', 't',  'o',
      'r', ' ', 's', 'e',    't', ' ',    't', 'o', ':', ' ', '%', '.',  '2',
      'f', ' ', 'N', '\xb7', 'm', '\xb7', 's', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T c_u[37] = {
      'K', '_', 'r', 'o',    'l', 'l', '_', 't', 'r', 'a',  'i', 'l', 'e',
      'r', ' ', 's', 'e',    't', ' ', 't', 'o', ':', ' ',  '%', '.', '2',
      'f', ' ', 'N', '\xb7', 'm', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T u[37] = {
      'K', '_', 'r', 'o',    'l', 'l', '_', 't', 'r', 'a',  'c', 't', 'o',
      'r', ' ', 's', 'e',    't', ' ', 't', 'o', ':', ' ',  '%', '.', '2',
      'f', ' ', 'N', '\xb7', 'm', '/', 'r', 'a', 'd', '\\', 'n'};
  static const char_T cv2[35] = {'h', '_', 'C', 'o', 'G', '_', 't',  'r', 'a',
                                 'c', 't', 'o', 'r', ' ', 's', 'e',  't', ' ',
                                 't', 'o', ':', ' ', '%', '.', '2',  'f', ' ',
                                 'm', 'e', 't', 'e', 'r', 's', '\\', 'n'};
  static const char_T cv3[35] = {'h', '_', 'C', 'o', 'G', '_', 't',  'r', 'a',
                                 'i', 'l', 'e', 'r', ' ', 's', 'e',  't', ' ',
                                 't', 'o', ':', ' ', '%', '.', '2',  'f', ' ',
                                 'm', 'e', 't', 'e', 'r', 's', '\\', 'n'};
  static const char_T cv4[35] = {
      'I', '_', 'r', 'o', 'l', 'l', '_',    't', 'r',    'a',  'c', 't',
      'o', 'r', ' ', 's', 'e', 't', ' ',    't', 'o',    ':',  ' ', '%',
      '.', '2', 'f', ' ', 'k', 'g', '\xb7', 'm', '\xb2', '\\', 'n'};
  static const char_T cv5[35] = {
      'I', '_', 'r', 'o', 'l', 'l', '_',    't', 'r',    'a',  'i', 'l',
      'e', 'r', ' ', 's', 'e', 't', ' ',    't', 'o',    ':',  ' ', '%',
      '.', '2', 'f', ' ', 'k', 'g', '\xb7', 'm', '\xb2', '\\', 'n'};
  static const char_T e_u[25] = {'d', 't', ' ', 's', 'e', 't',  ' ', 't', 'o',
                                 ':', ' ', '%', '.', '4', 'f',  ' ', 's', 'e',
                                 'c', 'o', 'n', 'd', 's', '\\', 'n'};
  static const char_T cv[13] = {'S', 'u', 'p', 'p', 'r', 'e', 's',
                                's', 'D', 'e', 'b', 'u', 'g'};
  static const char_T b[7] = {'t', 'r', 'a', 'c', 't', 'o', 'r'};
  static const char_T cv1[7] = {'f', 'p', 'r', 'i', 'n', 't', 'f'};
  static const char_T vehicleType[7] = {'t', 'r', 'a', 'c', 't', 'o', 'r'};
  static const char_T fc_tireModelFlag[6] = {'s', 'i', 'm', 'p', 'l', 'e'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  int32_T tmp_size[2];
  int32_T i;
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
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  /*  StabilityChecker_checkStability_wrapper - wrapper for
   * StabilityChecker.checkStability */
  /*  Initializes dummy vehicle objects and returns the stability flags */
  if (!checker_not_empty) {
    int32_T ret;
    /*  Minimal initialization of dependencies */
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
    /*         %% Constructor */
    /*  Constructor for ForceCalculator */
    /*  */
    /*  Preserves all original parameters, plus optional 'wheelSpeeds,
     * wheelRadius, wheelInertia' */
    /*  if given via varargin. */
    /*  Validate vehicleType using codegen friendly syntax */
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
    /*  --- Trailer Yaw Dynamics and Props --- */
    /*  Initialize tire parameters */
    /*         %% initializeTireParameters */
    /*  Initialize friction and rolling-resistance coefficients */
    /*  Initialize flat tire indices */
    /*  --- Wind Vector --- */
    b_st.site = &i_emlrtRSI;
    c_st.site = &h_emlrtRSI;
    WarningState_callWarning(&c_st);
    /*  --- Braking System --- */
    /*  Calculate axle masses and inertia */
    if (memcmp((char_T *)&vehicleType[0], (char_T *)&b[0], 7) == 0) {
      b_st.site = &j_emlrtRSI;
      /*         %% calculateAxleMasses */
      /*  Original logic for front/rear mass distribution */
      tmp_size[0] = 1;
      tmp_size[1] = 0;
      c_st.site = &l_emlrtRSI;
      sum(tmp_size);
      tmp_size[0] = 1;
      tmp_size[1] = 1;
      c_st.site = &m_emlrtRSI;
      sum(tmp_size);
      b_st.site = &k_emlrtRSI;
      /*         %% calculateInertia */
      /*  vector from CoG */
      c_st.site = &o_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      c_st.site = &o_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      c_st.site = &p_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      c_st.site = &p_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      c_st.site = &q_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      c_st.site = &q_emlrtRSI;
      d_st.site = &r_emlrtRSI;
      e_st.site = &s_emlrtRSI;
      /*  Add trailer inertia to Izz if tractor-trailer */
    }
    /*  --- Filter Setup --- */
    /*  Pre-allocate force buffers and filtered forces for moving average
     * (fixed-size) */
    /*  --- Optional wheel parameters via varargin --- */
    /*  default */
    st.site = &f_emlrtRSI;
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
    /*  * @param D_roll_tractor (Optional) Roll damping coefficient of the
     * tractor (N·m·s/rad). Defaults to 5,000 N·m·s/rad if not provided. */
    /*  * @param I_roll_tractor (Optional) Roll inertia of the tractor (kg·m²).
     * Defaults to 5,000 kg·m² if not provided. */
    /*  * @param K_roll_trailer (Optional) Roll stiffness of the trailer
     * (N·m/rad). Defaults to 150,000 N·m/rad if not provided. */
    /*  * @param D_roll_trailer (Optional) Roll damping coefficient of the
     * trailer (N·m·s/rad). Defaults to 4,000 N·m·s/rad if not provided. */
    /*  * @param I_roll_trailer (Optional) Roll inertia of the trailer (kg·m²).
     * Defaults to 8,000 kg·m² if not provided. */
    /*  * @param dt (Optional) Time step for integration (seconds). Defaults to
     * 0.01 seconds if not provided. */
    /*  * */
    /*  * @throws None */
    /*  _/ */
    /*  Initialize center of gravity heights */
    b_st.site = &t_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      e_st.site = &mb_emlrtRSI;
      f_st.site = &nb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0),
                               c_emlrt_marshallOut(&f_st, cv2),
                               b_emlrt_marshallOut(1.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &u_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      e_st.site = &mb_emlrtRSI;
      f_st.site = &nb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0),
                               c_emlrt_marshallOut(&f_st, cv3),
                               b_emlrt_marshallOut(2.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    /*  Initialize roll dynamics parameters for tractor */
    b_st.site = &v_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(&d_st, 37, m, &u[0]);
      emlrtAssign(&y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), y,
                               b_emlrt_marshallOut(200000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &w_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      b_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(&d_st, 39, m, &b_u[0]);
      emlrtAssign(&b_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), b_y,
                               b_emlrt_marshallOut(5000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &x_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      e_st.site = &mb_emlrtRSI;
      f_st.site = &nb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0),
                               c_emlrt_marshallOut(&f_st, cv4),
                               b_emlrt_marshallOut(5000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    /*  Initialize roll dynamics parameters for trailer */
    b_st.site = &y_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      c_y = NULL;
      m = emlrtCreateCharArray(2, &iv2[0]);
      emlrtInitCharArrayR2013a(&d_st, 37, m, &c_u[0]);
      emlrtAssign(&c_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), c_y,
                               b_emlrt_marshallOut(150000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &ab_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      d_y = NULL;
      m = emlrtCreateCharArray(2, &iv3[0]);
      emlrtInitCharArrayR2013a(&d_st, 60, m, &d_u[0]);
      emlrtAssign(&d_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), d_y,
                               b_emlrt_marshallOut(4000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &bb_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      e_st.site = &mb_emlrtRSI;
      f_st.site = &nb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0),
                               c_emlrt_marshallOut(&f_st, cv5),
                               b_emlrt_marshallOut(8000.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    b_st.site = &cb_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      e_y = NULL;
      m = emlrtCreateCharArray(2, &iv4[0]);
      emlrtInitCharArrayR2013a(&d_st, 25, m, &e_u[0]);
      emlrtAssign(&e_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       b_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), e_y,
                               b_emlrt_marshallOut(0.01), &e_emlrtMCI),
                       "<output of feval>");
    }
    /*  Assign ForceCalculator instance */
    /*  Initialize lateral accelerations and roll states */
    /*  Initialize original properties */
    /*  Assuming h_CoG refers to tractor's CoG */
    /*  Original lateralAcceleration remains, can be used if needed */
    /*  Add other original initializations as needed */
    st.site = &emlrtRSI;
    b_st.site = &fb_emlrtRSI;
    gobj_0.maxTorque = 500.0;
    gobj_0.engagementPercentage = 0.0;
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
    b_st.site = &gb_emlrtRSI;
    /*  Start at first gear */
    /*  Initialize to negative infinity */
    gobj_1.clutch = &gobj_0;
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
    b_st.site = &hb_emlrtRSI;
    /*  Initialize properties */
    for (i = 0; i < 7; i++) {
      gobj_2.forceCalculator.vehicleType[i] = vehicleType[i];
    }
    gobj_2.forceCalculator.frictionCoefficient = 0.8;
    for (i = 0; i < 5; i++) {
      gobj_2.forceCalculator.loadDistribution[i] = fc_loadDistribution[i];
    }
    gobj_2.forceCalculator.centerOfGravity[0] = 0.0;
    gobj_2.forceCalculator.centerOfGravity[1] = 0.0;
    gobj_2.forceCalculator.centerOfGravity[2] = 0.0;
    gobj_2.forceCalculator.inertia[0] = 0.0;
    gobj_2.forceCalculator.inertia[1] = 0.0;
    gobj_2.forceCalculator.inertia[2] = 0.0;
    gobj_2.forceCalculator.gravity = 9.81;
    gobj_2.forceCalculator.calculatedForces.hitch[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.hitch[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.hitch[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.hitchLateralForce = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_drag_global = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_side_global = 0.0;
    gobj_2.forceCalculator.calculatedForces.momentZ_wind = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction_force[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction_force[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.traction_force[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.M_z = 0.0;
    gobj_2.forceCalculator.calculatedForces.momentZ = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_y_total = 0.0;
    gobj_2.forceCalculator.calculatedForces.momentRoll = 0.0;
    gobj_2.forceCalculator.calculatedForces.totalForce[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.totalForce[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.totalForce[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_y_trailer = 0.0;
    gobj_2.forceCalculator.calculatedForces.momentZ_trailer = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_local[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_local[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_local[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_global[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_global[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.F_total_trailer_global[2] = 0.0;
    gobj_2.forceCalculator.calculatedForces.yawMoment_trailer = 0.0;
    gobj_2.forceCalculator.calculatedForces.momentRoll_trailer = 0.0;
    gobj_2.forceCalculator.calculatedForces.trailerPsi = 0.0;
    gobj_2.forceCalculator.calculatedForces.trailerOmega = 0.0;
    gobj_2.forceCalculator.calculatedForces.rolloverRiskIndex = 0.0;
    gobj_2.forceCalculator.calculatedForces.jackknifeRiskIndex = 0.0;
    gobj_2.forceCalculator.calculatedForces.braking[0] = 0.0;
    gobj_2.forceCalculator.calculatedForces.braking[1] = 0.0;
    gobj_2.forceCalculator.calculatedForces.braking[2] = 0.0;
    gobj_2.forceCalculator.tireModel.B = 10.0;
    gobj_2.forceCalculator.tireModel.C = 1.9;
    gobj_2.forceCalculator.tireModel.D = 1.0;
    gobj_2.forceCalculator.tireModel.E = 0.97;
    gobj_2.forceCalculator.numTrailerTires = 0.0;
    for (i = 0; i < 6; i++) {
      gobj_2.forceCalculator.tireModelFlag[i] = fc_tireModelFlag[i];
    }
    /*  [x; y] */
    /*  Yaw angle (theta) */
    gobj_2.mass = 10000.0;
    /*  m/s */
    for (i = 0; i < 7; i++) {
      gobj_2.vehicleType[i] = vehicleType[i];
    }
    /*  'tractor-trailer', 'tractor', or 'PassengerVehicle' */
    /*  Initialize roll stiffness and damping */
    /*  Initialize Transmission */
    gobj_2.transmission = &gobj_1;
    /*  Initialize simulation time */
    /*  Initialize trailer state if vehicleType is 'tractor-trailer' */
    ret = memcmp(&gobj_2.vehicleType[0], (char_T *)&b[0], 7);
    if (ret != 0) {
      b_st.site = &ib_emlrtRSI;
      b_error(&b_st, gobj_2.vehicleType);
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
    st.site = &g_emlrtRSI;
    /*  ================================================================ */
    /*  Constructor */
    /*  ================================================================ */
    b_st.site = &kb_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &jb_emlrtRSI;
      f_y = NULL;
      m = emlrtCreateCharArray(2, &iv5[0]);
      emlrtInitCharArrayR2013a(&d_st, 63, m, &f_u[0]);
      emlrtAssign(&f_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       c_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), f_y, &e_emlrtMCI),
                       "<output of feval>");
    }
    /*  Compute thresholds */
    b_st.site = &lb_emlrtRSI;
    /*  debugLog Conditionally prints debug messages based on global suppression
     * flag. */
    /*  If the appdata 'SuppressDebug' is true, messages are suppressed.
     * Otherwise printed via fprintf. */
    /*  Usage: debugLog('Value: %d\n', x); */
    c_st.site = &ob_emlrtRSI;
    if ((!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                isappdata(&c_st, b_emlrt_marshallOut(0.0),
                          d_emlrt_marshallOut(&c_st, cv), &b_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>")) ||
        (!c_emlrt_marshallIn(
            &c_st,
            c_coder_internal_ifWhileCondExt(
                &c_st,
                getappdata(&c_st, b_emlrt_marshallOut(0.0),
                           d_emlrt_marshallOut(&c_st, cv), &d_emlrtMCI),
                &c_emlrtMCI),
            "<output of coder.internal.ifWhileCondExtrinsic>"))) {
      c_st.site = &db_emlrtRSI;
      d_st.site = &eb_emlrtRSI;
      g_y = NULL;
      m = emlrtCreateCharArray(2, &iv6[0]);
      emlrtInitCharArrayR2013a(&d_st, 79, m, &g_u[0]);
      emlrtAssign(&g_y, m);
      e_st.site = &mb_emlrtRSI;
      emlrt_marshallIn(&e_st,
                       d_feval(&e_st, emlrt_marshallOut(&e_st, cv1),
                               b_emlrt_marshallOut(1.0), g_y,
                               b_emlrt_marshallOut(0.3805063771123649),
                               b_emlrt_marshallOut(0.0),
                               b_emlrt_marshallOut(0.0),
                               b_emlrt_marshallOut(0.0), &e_emlrtMCI),
                       "<output of feval>");
    }
    /*  Initialize trailer yaw circular buffer */
    memset(&checker.trailerYawBuffer.data[0], 0, 20U * sizeof(real_T));
    checker_not_empty = true;
  }
  st.site = &d_emlrtRSI;
  /*  ================================================================ */
  /*  CHECK STABILITY */
  /*  ================================================================ */
  /*  ------------------------------------------ */
  /*  1) If speed is below a certain threshold, */
  /*     reset all flags to false & scores to 0. */
  /*  ------------------------------------------ */
  /*  only horizontal? */
  /*  e.g., 2 m/s */
  /*  Reset everything */
  /*  ================================================================ */
  /*  GET STABILITY FLAGS */
  /*  ================================================================ */
  *isWiggling = false;
  *isRollover = false;
  *isSkidding = false;
  *isJackknife = false;
}

void d_StabilityChecker_checkStabili(void)
{
  checker_not_empty = false;
}

/* End of code generation (StabilityChecker_checkStability_wrapper.c) */
