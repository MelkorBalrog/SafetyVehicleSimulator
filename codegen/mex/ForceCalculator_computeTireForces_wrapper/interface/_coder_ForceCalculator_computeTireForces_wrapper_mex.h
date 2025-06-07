/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * _coder_ForceCalculator_computeTireForces_wrapper_mex.h
 *
 * Code generation for function
 * '_coder_ForceCalculator_computeTireForces_wrapper_mex'
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
void ForceCalculator_computeTireForces_wrapper_mexFunction(
    int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const mxArray *prhs[5]);

MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

/* End of code generation
 * (_coder_ForceCalculator_computeTireForces_wrapper_mex.h) */
