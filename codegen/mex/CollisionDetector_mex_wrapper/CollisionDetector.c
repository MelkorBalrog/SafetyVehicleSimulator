/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector.c
 *
 * Code generation for function 'CollisionDetector'
 *
 */

/* Include files */
#include "CollisionDetector.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_or(boolean_T in1_data[], int32_T in1_size[2], const boolean_T in2_data[],
          const int32_T in2_size[2])
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  boolean_T b_in1_data[8];
  if (in2_size[1] == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = in2_size[1];
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_1 = (in2_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = (in1_data[i * stride_0_1] || in2_data[i * stride_1_1]);
  }
  in1_size[0] = 1;
  in1_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    memcpy(&in1_data[0], &b_in1_data[0], (uint32_T)loop_ub * sizeof(boolean_T));
  }
}

void lt(boolean_T in1_data[], int32_T in1_size[2], const real_T in2_data[],
        const int32_T in2_size[2], const real_T in3_data[],
        const int32_T in3_size[2])
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in1_size[0] = 1;
  if (in3_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in3_size[1];
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_1] < in3_data[i * stride_1_1]);
  }
}

/* End of code generation (CollisionDetector.c) */
