/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * ForceCalculator_computeTireForces_wrapper.h
 *
 * Code generation for function 'ForceCalculator_computeTireForces_wrapper'
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
void ForceCalculator_computeTireForces_wrapper(const emlrtStack *sp,
                                               const real_T loads[4],
                                               const real_T contactAreas[4],
                                               real_T u, real_T v, real_T r,
                                               real_T *F_y_total, real_T *M_z);

void d_ForceCalculator_computeTireFo(void);

/* End of code generation (ForceCalculator_computeTireForces_wrapper.h) */
