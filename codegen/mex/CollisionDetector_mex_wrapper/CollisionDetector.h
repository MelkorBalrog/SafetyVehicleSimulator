/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector.h
 *
 * Code generation for function 'CollisionDetector'
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
void b_or(boolean_T in1_data[], int32_T in1_size[2], const boolean_T in2_data[],
          const int32_T in2_size[2]);

void lt(boolean_T in1_data[], int32_T in1_size[2], const real_T in2_data[],
        const int32_T in2_size[2], const real_T in3_data[],
        const int32_T in3_size[2]);

/* End of code generation (CollisionDetector.h) */
