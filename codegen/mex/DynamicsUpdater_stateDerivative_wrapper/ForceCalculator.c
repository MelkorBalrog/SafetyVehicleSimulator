/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * ForceCalculator.c
 *
 * Code generation for function 'ForceCalculator'
 *
 */

/* Include files */
#include "ForceCalculator.h"
#include "DynamicsUpdater_stateDerivative_wrapper_internal_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo kb_emlrtRSI = {
    1039,                              /* lineNo */
    "ForceCalculator/calculateForces", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m" /* pathName */
};

static emlrtDCInfo emlrtDCI = {
    1263,                                       /* lineNo */
    45,                                         /* colNo */
    "ForceCalculator/applyMovingAverageFilter", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m", /* pName */
    1                                                /* checkKind */
};

static emlrtBCInfo emlrtBCI = {
    1,                                          /* iFirst */
    5,                                          /* iLast */
    1263,                                       /* lineNo */
    45,                                         /* colNo */
    "buffStruct.data",                          /* aName */
    "ForceCalculator/applyMovingAverageFilter", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m", /* pName */
    0                                                /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    1264,                                       /* lineNo */
    33,                                         /* colNo */
    "ForceCalculator/applyMovingAverageFilter", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m", /* pName */
    1                                                /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                                          /* iFirst */
    5,                                          /* iLast */
    1264,                                       /* lineNo */
    33,                                         /* colNo */
    "buffStruct.data",                          /* aName */
    "ForceCalculator/applyMovingAverageFilter", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\ForceCalculator.m", /* pName */
    3                                                /* checkKind */
};

/* Function Definitions */
void ForceCalculator_calculateForces(const emlrtStack *sp, ForceCalculator *obj)
{
  emlrtStack st;
  struct_T buffStruct;
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T i;
  int32_T i1;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  /*         %% calculateForces */
  /*  Calculate all relevant forces based on current vehicle state */
  /*  */
  /*  Preserves original logic, and at the END we add optional rollover &
   * jackknife calculations. */
  /*         %% updatePacejkaAttributes */
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(obj->velocity[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(obj->velocity[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  if (y > 30.0) {
    scale = 0.95;
  } else {
    scale = 1.0;
  }
  obj->B_tires *= scale;
  obj->C_tires *= scale;
  obj->D_tires *= scale;
  obj->E_tires *= scale;
  /*  Adjust for flat tires */
  /*  Gravity in vehicle frame */
  /*  default */
  /* --------------------------------------------- */
  /*  IF BRANCH: (tractor) & small acceleration */
  /* --------------------------------------------- */
  /*             %% -- OPTIONAL Rollover Risk & Jackknife Risk Calculation
   * (UPDATED) -- */
  /*  1) Rollover Risk */
  /*  Extract rollRate (if present) */
  /*  Extract rollAngle (if present) */
  /*  Get total lateral force in vehicle frame */
  /*  Also add side force in vehicle frame (if it exists) */
  /*  Example new constants */
  /*  roll-rate weight */
  /*  roll-angle weight */
  /*  Normalized lateral moment ratio relative to half track width */
  obj->calculatedForces.rolloverRiskIndex =
      muDoubleScalarAbs(obj->calculatedForces.F_y_total +
                        -muDoubleScalarSin(obj->orientation) *
                            obj->calculatedForces.F_side_global) *
      obj->h_CoG / (obj->vehicleMass * obj->gravity * (obj->trackWidth / 2.0));
  /*  2) Jackknife Risk */
  /*  Hitch angle = difference between trailer yaw and tractor yaw */
  /*  Example new constants */
  /*  5000 N => ~1.0 scale */
  /*             %% Finally apply moving average filter */
  st.site = &kb_emlrtRSI;
  /*         %% applyMovingAverageFilter */
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.hitchLateralForce;
  buffStruct.index = obj->forceBuffers.hitchLateralForce.index + 1.0;
  if (obj->forceBuffers.hitchLateralForce.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = 0.0;
  buffStruct.sum = obj->forceBuffers.hitchLateralForce.sum -
                   obj->forceBuffers.hitchLateralForce.data[i1 - 1];
  if (obj->forceBuffers.hitchLateralForce.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.hitchLateralForce.count + 1.0;
  }
  obj->forceBuffers.hitchLateralForce = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.F_drag_global;
  buffStruct.index = obj->forceBuffers.F_drag_global.index + 1.0;
  if (obj->forceBuffers.F_drag_global.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.F_drag_global;
  buffStruct.sum = (obj->forceBuffers.F_drag_global.sum +
                    obj->calculatedForces.F_drag_global) -
                   obj->forceBuffers.F_drag_global.data[i1 - 1];
  if (obj->forceBuffers.F_drag_global.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.F_drag_global.count + 1.0;
  }
  obj->forceBuffers.F_drag_global = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.F_side_global;
  buffStruct.index = obj->forceBuffers.F_side_global.index + 1.0;
  if (obj->forceBuffers.F_side_global.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.F_side_global;
  buffStruct.sum = (obj->forceBuffers.F_side_global.sum +
                    obj->calculatedForces.F_side_global) -
                   obj->forceBuffers.F_side_global.data[i1 - 1];
  if (obj->forceBuffers.F_side_global.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.F_side_global.count + 1.0;
  }
  obj->forceBuffers.F_side_global = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.momentZ_wind;
  buffStruct.index = obj->forceBuffers.momentZ_wind.index + 1.0;
  if (obj->forceBuffers.momentZ_wind.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.momentZ_wind;
  buffStruct.sum = (obj->forceBuffers.momentZ_wind.sum +
                    obj->calculatedForces.momentZ_wind) -
                   obj->forceBuffers.momentZ_wind.data[i1 - 1];
  if (obj->forceBuffers.momentZ_wind.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.momentZ_wind.count + 1.0;
  }
  obj->forceBuffers.momentZ_wind = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.M_z;
  buffStruct.index = obj->forceBuffers.M_z.index + 1.0;
  if (obj->forceBuffers.M_z.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.M_z;
  buffStruct.sum = (obj->forceBuffers.M_z.sum + obj->calculatedForces.M_z) -
                   obj->forceBuffers.M_z.data[i1 - 1];
  if (obj->forceBuffers.M_z.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.M_z.count + 1.0;
  }
  obj->forceBuffers.M_z = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.momentZ;
  buffStruct.index = obj->forceBuffers.momentZ.index + 1.0;
  if (obj->forceBuffers.momentZ.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.momentZ;
  buffStruct.sum =
      (obj->forceBuffers.momentZ.sum + obj->calculatedForces.momentZ) -
      obj->forceBuffers.momentZ.data[i1 - 1];
  if (obj->forceBuffers.momentZ.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.momentZ.count + 1.0;
  }
  obj->forceBuffers.momentZ = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.F_y_total;
  buffStruct.index = obj->forceBuffers.F_y_total.index + 1.0;
  if (obj->forceBuffers.F_y_total.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.F_y_total;
  buffStruct.sum =
      (obj->forceBuffers.F_y_total.sum + obj->calculatedForces.F_y_total) -
      obj->forceBuffers.F_y_total.data[i1 - 1];
  if (obj->forceBuffers.F_y_total.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.F_y_total.count + 1.0;
  }
  obj->forceBuffers.F_y_total = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.momentRoll;
  buffStruct.index = obj->forceBuffers.momentRoll.index + 1.0;
  if (obj->forceBuffers.momentRoll.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.momentRoll;
  buffStruct.sum =
      (obj->forceBuffers.momentRoll.sum + obj->calculatedForces.momentRoll) -
      obj->forceBuffers.momentRoll.data[i1 - 1];
  if (obj->forceBuffers.momentRoll.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.momentRoll.count + 1.0;
  }
  obj->forceBuffers.momentRoll = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.F_y_trailer;
  buffStruct.index = obj->forceBuffers.F_y_trailer.index + 1.0;
  if (obj->forceBuffers.F_y_trailer.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.F_y_trailer;
  buffStruct.sum =
      (obj->forceBuffers.F_y_trailer.sum + obj->calculatedForces.F_y_trailer) -
      obj->forceBuffers.F_y_trailer.data[i1 - 1];
  if (obj->forceBuffers.F_y_trailer.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.F_y_trailer.count + 1.0;
  }
  obj->forceBuffers.F_y_trailer = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.momentZ_trailer;
  buffStruct.index = obj->forceBuffers.momentZ_trailer.index + 1.0;
  if (obj->forceBuffers.momentZ_trailer.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.momentZ_trailer;
  buffStruct.sum = (obj->forceBuffers.momentZ_trailer.sum +
                    obj->calculatedForces.momentZ_trailer) -
                   obj->forceBuffers.momentZ_trailer.data[i1 - 1];
  if (obj->forceBuffers.momentZ_trailer.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.momentZ_trailer.count + 1.0;
  }
  obj->forceBuffers.momentZ_trailer = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.yawMoment_trailer;
  buffStruct.index = obj->forceBuffers.yawMoment_trailer.index + 1.0;
  if (obj->forceBuffers.yawMoment_trailer.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.yawMoment_trailer;
  buffStruct.sum = (obj->forceBuffers.yawMoment_trailer.sum +
                    obj->calculatedForces.yawMoment_trailer) -
                   obj->forceBuffers.yawMoment_trailer.data[i1 - 1];
  if (obj->forceBuffers.yawMoment_trailer.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.yawMoment_trailer.count + 1.0;
  }
  obj->forceBuffers.yawMoment_trailer = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.momentRoll_trailer;
  buffStruct.index = obj->forceBuffers.momentRoll_trailer.index + 1.0;
  if (obj->forceBuffers.momentRoll_trailer.index + 1.0 >
      obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.momentRoll_trailer;
  buffStruct.sum = (obj->forceBuffers.momentRoll_trailer.sum +
                    obj->calculatedForces.momentRoll_trailer) -
                   obj->forceBuffers.momentRoll_trailer.data[i1 - 1];
  if (obj->forceBuffers.momentRoll_trailer.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.momentRoll_trailer.count + 1.0;
  }
  obj->forceBuffers.momentRoll_trailer = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.trailerPsi;
  buffStruct.index = obj->forceBuffers.trailerPsi.index + 1.0;
  if (obj->forceBuffers.trailerPsi.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.trailerPsi;
  buffStruct.sum =
      (obj->forceBuffers.trailerPsi.sum + obj->calculatedForces.trailerPsi) -
      obj->forceBuffers.trailerPsi.data[i1 - 1];
  if (obj->forceBuffers.trailerPsi.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.trailerPsi.count + 1.0;
  }
  obj->forceBuffers.trailerPsi = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.trailerOmega;
  buffStruct.index = obj->forceBuffers.trailerOmega.index + 1.0;
  if (obj->forceBuffers.trailerOmega.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.trailerOmega;
  buffStruct.sum = (obj->forceBuffers.trailerOmega.sum +
                    obj->calculatedForces.trailerOmega) -
                   obj->forceBuffers.trailerOmega.data[i1 - 1];
  if (obj->forceBuffers.trailerOmega.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.trailerOmega.count + 1.0;
  }
  obj->forceBuffers.trailerOmega = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.rolloverRiskIndex;
  buffStruct.index = obj->forceBuffers.rolloverRiskIndex.index + 1.0;
  if (obj->forceBuffers.rolloverRiskIndex.index + 1.0 > obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = obj->calculatedForces.rolloverRiskIndex;
  buffStruct.sum = (obj->forceBuffers.rolloverRiskIndex.sum +
                    obj->calculatedForces.rolloverRiskIndex) -
                   obj->forceBuffers.rolloverRiskIndex.data[i1 - 1];
  if (obj->forceBuffers.rolloverRiskIndex.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.rolloverRiskIndex.count + 1.0;
  }
  obj->forceBuffers.rolloverRiskIndex = buffStruct;
  /*  Moving average buffer update for scalar forces */
  buffStruct = obj->forceBuffers.jackknifeRiskIndex;
  buffStruct.index = obj->forceBuffers.jackknifeRiskIndex.index + 1.0;
  if (obj->forceBuffers.jackknifeRiskIndex.index + 1.0 >
      obj->filterWindowSize) {
    buffStruct.index = 1.0;
  }
  i = (int32_T)muDoubleScalarFloor(buffStruct.index);
  if (buffStruct.index != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &emlrtDCI, &st);
  }
  b = ((buffStruct.index < 1.0) || (buffStruct.index > 5.0));
  if (b) {
    emlrtDynamicBoundsCheckR2012b((int32_T)buffStruct.index, 1, 5, &emlrtBCI,
                                  &st);
  }
  i1 = (int32_T)buffStruct.index;
  if (i1 != i) {
    emlrtIntegerCheckR2012b(buffStruct.index, &b_emlrtDCI, &st);
  }
  if (b) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, 5, &b_emlrtBCI, &st);
  }
  buffStruct.data[i1 - 1] = 0.0;
  buffStruct.sum = obj->forceBuffers.jackknifeRiskIndex.sum -
                   obj->forceBuffers.jackknifeRiskIndex.data[i1 - 1];
  if (obj->forceBuffers.jackknifeRiskIndex.count < obj->filterWindowSize) {
    buffStruct.count = obj->forceBuffers.jackknifeRiskIndex.count + 1.0;
  }
  obj->forceBuffers.jackknifeRiskIndex = buffStruct;
}

/* End of code generation (ForceCalculator.c) */
