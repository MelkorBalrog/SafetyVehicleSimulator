/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * DynamicsUpdater_stateDerivative_wrapper_internal_types.h
 *
 * Code generation for function 'DynamicsUpdater_stateDerivative_wrapper'
 *
 */

#pragma once

/* Include files */
#include "DynamicsUpdater_stateDerivative_wrapper_types.h"
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T data[5];
  real_T sum;
  real_T index;
  real_T count;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  real_T F_drag_global;
  real_T F_side_global;
  real_T momentZ_wind;
  real_T M_z;
  real_T momentZ;
  real_T F_y_total;
  real_T momentRoll;
  real_T totalForce[3];
  real_T F_y_trailer;
  real_T momentZ_trailer;
  real_T yawMoment_trailer;
  real_T momentRoll_trailer;
  real_T trailerPsi;
  real_T trailerOmega;
  real_T rolloverRiskIndex;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  struct_T hitchLateralForce;
  struct_T F_drag_global;
  struct_T F_side_global;
  struct_T momentZ_wind;
  struct_T M_z;
  struct_T momentZ;
  struct_T F_y_total;
  struct_T momentRoll;
  struct_T F_y_trailer;
  struct_T momentZ_trailer;
  struct_T yawMoment_trailer;
  struct_T momentRoll_trailer;
  struct_T trailerPsi;
  struct_T trailerOmega;
  struct_T rolloverRiskIndex;
  struct_T jackknifeRiskIndex;
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_ForceCalculator
#define typedef_ForceCalculator
typedef struct {
  char_T vehicleType[7];
  real_T vehicleMass;
  real_T velocity[3];
  real_T loadDistribution[5];
  real_T h_CoG;
  real_T inertia[3];
  real_T gravity;
  b_struct_T calculatedForces;
  real_T orientation;
  real_T trackWidth;
  real_T B_tires;
  real_T C_tires;
  real_T D_tires;
  real_T E_tires;
  c_struct_T forceBuffers;
  real_T filterWindowSize;
} ForceCalculator;
#endif /* typedef_ForceCalculator */

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtEqualityCheckInfo
#define typedef_rtEqualityCheckInfo
typedef struct {
  int32_T nDims;
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtEqualityCheckInfo;
#endif /* typedef_rtEqualityCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation
 * (DynamicsUpdater_stateDerivative_wrapper_internal_types.h) */
