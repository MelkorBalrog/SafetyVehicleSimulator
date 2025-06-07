/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * DynamicsUpdater_stateDerivative_wrapper.h
 *
 * Code generation for function 'DynamicsUpdater_stateDerivative_wrapper'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void DynamicsUpdater_stateDerivative_wrapper(const emlrtStack *sp,
                                             const real_T stateVec[8],
                                             real_T dydt[8],
                                             real_T accelerations[2]);

void d_DynamicsUpdater_stateDerivati(void);

/* End of code generation (DynamicsUpdater_stateDerivative_wrapper.h) */
