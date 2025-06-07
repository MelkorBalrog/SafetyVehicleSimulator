/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * sumMatrixIncludeNaN.c
 *
 * Code generation for function 'sumMatrixIncludeNaN'
 *
 */

/* Include files */
#include "sumMatrixIncludeNaN.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
real_T sumColumnB(const real_T x[4])
{
  return ((x[0] + x[1]) + x[2]) + x[3];
}

/* End of code generation (sumMatrixIncludeNaN.c) */
