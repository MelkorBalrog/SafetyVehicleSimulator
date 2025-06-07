/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity_CalculateSeverity_wrapper.h
 *
 * Code generation for function
 * 'VehicleCollisionSeverity_CalculateSeverity_wrapper'
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
void VehicleCollisionSeverity_CalculateSeverity_wrapper(
    const emlrtStack *sp, real_T *dV1, real_T *dV2, char_T severity1_data[],
    int32_T severity1_size[2], char_T severity2_data[],
    int32_T severity2_size[2]);

/* End of code generation (VehicleCollisionSeverity_CalculateSeverity_wrapper.h)
 */
