/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * StabilityChecker_checkStability_wrapper.h
 *
 * Code generation for function 'StabilityChecker_checkStability_wrapper'
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
void StabilityChecker_checkStability_wrapper(const emlrtStack *sp,
                                             boolean_T *isWiggling,
                                             boolean_T *isRollover,
                                             boolean_T *isSkidding,
                                             boolean_T *isJackknife);

void d_StabilityChecker_checkStabili(void);

/* End of code generation (StabilityChecker_checkStability_wrapper.h) */
