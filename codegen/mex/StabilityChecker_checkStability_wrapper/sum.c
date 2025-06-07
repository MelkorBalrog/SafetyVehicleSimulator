/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
real_T sum(const int32_T x_size[2])
{
  real_T y;
  if (x_size[1] == 0) {
    y = 0.0;
  } else {
    y = 98100.0;
  }
  return y;
}

/* End of code generation (sum.c) */
