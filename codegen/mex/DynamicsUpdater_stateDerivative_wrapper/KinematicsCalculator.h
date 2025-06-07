/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * KinematicsCalculator.h
 *
 * Code generation for function 'KinematicsCalculator'
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
real_T c_KinematicsCalculator_Kinemati(
    const emlrtStack *sp, real_T *obj_h_CoG_trailer, real_T *obj_K_roll_tractor,
    real_T *obj_D_roll_tractor, real_T *obj_I_roll_tractor,
    real_T *obj_K_roll_trailer, real_T *obj_D_roll_trailer,
    real_T *obj_I_roll_trailer, real_T *obj_dt);

/* End of code generation (KinematicsCalculator.h) */
