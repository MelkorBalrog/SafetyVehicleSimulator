/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector_mex_wrapper.h
 *
 * Code generation for function 'CollisionDetector_mex_wrapper'
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
boolean_T CollisionDetector_mex_wrapper(const emlrtStack *sp,
                                        const real_T corners1[8],
                                        const real_T corners2[8]);

emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

/* End of code generation (CollisionDetector_mex_wrapper.h) */
