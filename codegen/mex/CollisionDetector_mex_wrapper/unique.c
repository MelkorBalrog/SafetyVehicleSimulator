/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * unique.c
 *
 * Code generation for function 'unique'
 *
 */

/* Include files */
#include "unique.h"
#include "rt_nonfinite.h"
#include "sortLE.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo b_emlrtRTEI = {
    337,           /* lineNo */
    1,             /* colNo */
    "unique_rows", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

/* Function Definitions */
void unique_rows(const emlrtStack *sp, const real_T a_data[],
                 const int32_T a_size[2], real_T b_data[], int32_T b_size[2])
{
  int32_T idx_data[8];
  int32_T iwork_data[8];
  int32_T j;
  int32_T k;
  int32_T pEnd;
  int32_T qEnd;
  if (a_size[0] == 0) {
    b_size[0] = 0;
    b_size[1] = 2;
  } else {
    real_T ycol_data[8];
    int32_T b_i;
    int32_T b_k;
    int32_T i;
    int32_T loop_ub;
    int32_T n;
    int32_T p;
    int32_T q;
    loop_ub = a_size[0];
    b_size[0] = a_size[0];
    pEnd = a_size[0] << 1;
    memcpy(&b_data[0], &a_data[0], (uint32_T)pEnd * sizeof(real_T));
    n = a_size[0] + 1;
    memset(&idx_data[0], 0, (uint32_T)loop_ub * sizeof(int32_T));
    pEnd = a_size[0] - 1;
    for (k = 1; k <= pEnd; k += 2) {
      if (sortLE(a_data, a_size, k, k + 1)) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if (((uint32_T)a_size[0] & 1U) != 0U) {
      idx_data[a_size[0] - 1] = a_size[0];
    }
    i = 2;
    while (i < n - 1) {
      int32_T b_j;
      int32_T i2;
      i2 = i << 1;
      b_j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int32_T kEnd;
        p = b_j;
        q = pEnd;
        qEnd = b_j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        b_k = 0;
        kEnd = qEnd - b_j;
        while (b_k < kEnd) {
          int32_T i1;
          b_i = idx_data[q - 1];
          i1 = idx_data[p - 1];
          if (sortLE(a_data, a_size, i1, b_i)) {
            iwork_data[b_k] = i1;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[b_k] = b_i;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[p - 1];
                p++;
              }
            }
          }
          b_k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx_data[(b_j + k) - 1] = iwork_data[k];
        }
        b_j = qEnd;
      }
      i = i2;
    }
    for (j = 0; j < 2; j++) {
      for (k = 0; k < loop_ub; k++) {
        ycol_data[k] = b_data[(idx_data[k] + b_size[0] * j) - 1];
      }
      for (k = 0; k < loop_ub; k++) {
        b_data[k + b_size[0] * j] = ycol_data[k];
      }
    }
    pEnd = -1;
    p = 0;
    while (p + 1 <= loop_ub) {
      q = p;
      int32_T exitg1;
      do {
        exitg1 = 0;
        p++;
        if (p + 1 > loop_ub) {
          exitg1 = 1;
        } else {
          boolean_T b_p;
          boolean_T exitg2;
          b_p = false;
          b_k = 0;
          exitg2 = false;
          while ((!exitg2) && (b_k < 2)) {
            b_i = b_size[0] * b_k;
            if (b_data[q + b_i] != b_data[p + b_i]) {
              b_p = true;
              exitg2 = true;
            } else {
              b_k++;
            }
          }
          if (b_p) {
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
      pEnd++;
      b_data[pEnd] = b_data[q];
      b_data[pEnd + b_size[0]] = b_data[q + b_size[0]];
    }
    if (pEnd + 1 > a_size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &b_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (pEnd + 1 < 1) {
      pEnd = 0;
    } else {
      pEnd++;
    }
    for (k = 0; k < 2; k++) {
      for (j = 0; j < pEnd; j++) {
        b_data[j + pEnd * k] = b_data[j + b_size[0] * k];
      }
    }
    b_size[0] = pEnd;
    b_size[1] = 2;
  }
}

/* End of code generation (unique.c) */
