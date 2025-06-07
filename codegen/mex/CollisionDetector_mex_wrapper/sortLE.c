/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * sortLE.c
 *
 * Code generation for function 'sortLE'
 *
 */

/* Include files */
#include "sortLE.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
boolean_T sortLE(const real_T v_data[], const int32_T v_size[2], int32_T idx1,
                 int32_T idx2)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    real_T v1;
    real_T v2;
    int32_T v1_tmp;
    boolean_T isnanv2;
    v1_tmp = v_size[0] * k;
    v1 = v_data[(idx1 + v1_tmp) - 1];
    v2 = v_data[(idx2 + v1_tmp) - 1];
    isnanv2 = muDoubleScalarIsNaN(v2);
    if ((v1 == v2) || (muDoubleScalarIsNaN(v1) && isnanv2)) {
      k++;
    } else {
      if ((!(v1 <= v2)) && (!isnanv2)) {
        p = false;
      }
      exitg1 = true;
    }
  }
  return p;
}

/* End of code generation (sortLE.c) */
