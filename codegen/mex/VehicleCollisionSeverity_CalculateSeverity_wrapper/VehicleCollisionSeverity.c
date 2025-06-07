/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity.c
 *
 * Code generation for function 'VehicleCollisionSeverity'
 *
 */

/* Include files */
#include "VehicleCollisionSeverity.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T HeadOnCollision[8];
  real_T RearEndCollision[8];
  real_T SideCollision[8];
  real_T ObliqueCollision[8];
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  struct_T LowerBound;
  struct_T HigherBound;
  struct_T Average;
} b_struct_T;
#endif /* typedef_b_struct_T */

/* Variable Definitions */
static b_struct_T KE_tables;

static emlrtRSInfo f_emlrtRSI = {
    241,                                       /* lineNo */
    "VehicleCollisionSeverity/get_thresholds", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    242,                                       /* lineNo */
    "VehicleCollisionSeverity/get_thresholds", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    62,       /* lineNo */
    "strrep", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strrep.m" /* pathName
                                                                          */
};

static emlrtRSInfo i_emlrtRSI = {
    87,                      /* lineNo */
    "compute_str_or_length", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strrep.m" /* pathName
                                                                          */
};

static emlrtRTEInfo emlrtRTEI = {
    86,                      /* lineNo */
    5,                       /* colNo */
    "compute_str_or_length", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\strfun\\strrep.m" /* pName
                                                                          */
};

/* Function Definitions */
void c_VehicleCollisionSeverity_get_(const emlrtStack *sp,
                                     const char_T collision_type_data[],
                                     real_T thresholds[8])
{
  static const char_T cv1[16] = {'R', 'e', 'a', 'r', 'E', 'n', 'd', 'C',
                                 'o', 'l', 'l', 'i', 's', 'i', 'o', 'n'};
  static const char_T b_cv[15] = {'H', 'e', 'a', 'd', 'O', 'n', 'C', 'o',
                                  'l', 'l', 'i', 's', 'i', 'o', 'n'};
  static const char_T cv2[13] = {'S', 'i', 'd', 'e', 'C', 'o', 'l',
                                 'l', 'i', 's', 'i', 'o', 'n'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T b_copyfrom;
  int32_T b_in_idx;
  int32_T copyfrom;
  int32_T exitg1;
  int32_T in_idx;
  int32_T out_idx;
  int32_T out_len;
  int32_T patt_idx;
  int32_T tmp_in_idx;
  char_T ct_data[17];
  char_T source_str_data[17];
  boolean_T result;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  /* /_* */
  /*  * @brief Retrieves the severity thresholds based on boundary type,
   * collision type, and vehicle mass. */
  /*  * */
  /*  * @param bound_type_str Boundary type string ('LowerBound', 'HigherBound',
   * or 'Average'). */
  /*  * @param collision_type Type of collision ('Head-On Collision', 'Rear-End
   * Collision', 'Side Collision', or 'Oblique Collision'). */
  /*  * @param vehicle_mass Mass of the vehicle (kg). */
  /*  * */
  /*  * @return thresholds Nx2 matrix of [min, max] delta-V values in kph. */
  /*  * */
  /*  * @throws Error if an invalid boundary type or collision type is provided.
   */
  /*  _/ */
  /*  Retrieve the severity thresholds based on bound type, collision type, and
   * vehicle mass */
  /*  */
  /*  Parameters: */
  /*    bound_type_str - 'LowerBound', 'HigherBound', or 'Average' */
  /*    collision_type - 'Head-On Collision', 'Rear-End Collision', 'Side
   * Collision', or 'Oblique Collision' */
  /*    vehicle_mass - Mass of the vehicle (kg) */
  /*  */
  /*  Returns: */
  /*    thresholds - Nx2 matrix of [min, max] delta-V values in kph */
  /*  Vectorized delta-V thresholds lookup based on SAE J2980 tables */
  /*  MATLAB Coder does not allow dynamic field names, so select the */
  /*  appropriate table explicitly using switch statements */
  st.site = &f_emlrtRSI;
  b_st.site = &h_emlrtRSI;
  in_idx = 0;
  copyfrom = 1;
  out_idx = 0;
  b_in_idx = 0;
  b_copyfrom = 1;
  out_len = 0;
  while (b_in_idx < 17) {
    b_in_idx++;
    patt_idx = 1;
    tmp_in_idx = b_in_idx;
    if (collision_type_data[b_in_idx - 1] == '-') {
      tmp_in_idx = b_in_idx + 1;
      patt_idx = 2;
    }
    if (patt_idx > 1) {
      b_copyfrom = tmp_in_idx;
    } else if (b_in_idx >= b_copyfrom) {
      out_len++;
    }
  }
  if (out_len > 17) {
    emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  c_st.site = &i_emlrtRSI;
  if ((int8_T)out_len < 0) {
    emlrtNonNegativeCheckR2012b((int8_T)out_len, &emlrtDCI, &c_st);
  }
  patt_idx = (int8_T)out_len;
  while (in_idx < 17) {
    char_T c;
    in_idx++;
    b_in_idx = 1;
    b_copyfrom = in_idx;
    c = collision_type_data[in_idx - 1];
    if (c == '-') {
      b_copyfrom = in_idx + 1;
      b_in_idx = 2;
    }
    if (b_in_idx > 1) {
      copyfrom = b_copyfrom;
    } else if (in_idx >= copyfrom) {
      ct_data[out_idx] = c;
      out_idx++;
    }
  }
  st.site = &g_emlrtRSI;
  if ((int8_T)out_len != 0) {
    int32_T b_out_idx;
    b_st.site = &h_emlrtRSI;
    memcpy(&source_str_data[0], &ct_data[0],
           (uint32_T)patt_idx * sizeof(char_T));
    copyfrom = 0;
    out_idx = 1;
    b_out_idx = 0;
    b_in_idx = 0;
    b_copyfrom = 1;
    patt_idx = 0;
    while (b_in_idx < (int8_T)out_len) {
      b_in_idx++;
      tmp_in_idx = 1;
      in_idx = b_in_idx;
      if ((b_in_idx <= (int8_T)out_len) && (ct_data[b_in_idx - 1] == ' ')) {
        in_idx = b_in_idx + 1;
        tmp_in_idx = 2;
      }
      if (tmp_in_idx > 1) {
        b_copyfrom = in_idx;
      } else if (b_in_idx >= b_copyfrom) {
        patt_idx++;
      }
    }
    if (patt_idx > (int8_T)out_len) {
      emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    c_st.site = &i_emlrtRSI;
    if ((int8_T)patt_idx < 0) {
      emlrtNonNegativeCheckR2012b((int8_T)patt_idx, &emlrtDCI, &c_st);
    }
    patt_idx = (int8_T)patt_idx;
    while (copyfrom < (int8_T)out_len) {
      copyfrom++;
      b_in_idx = 1;
      b_copyfrom = copyfrom;
      if ((copyfrom <= (int8_T)out_len) &&
          (source_str_data[copyfrom - 1] == ' ')) {
        b_copyfrom = copyfrom + 1;
        b_in_idx = 2;
      }
      if (b_in_idx > 1) {
        out_idx = b_copyfrom;
      } else if (copyfrom >= out_idx) {
        ct_data[b_out_idx] = source_str_data[copyfrom - 1];
        b_out_idx++;
      }
    }
  }
  result = false;
  if (patt_idx == 15) {
    b_in_idx = 0;
    do {
      exitg1 = 0;
      if (b_in_idx < 15) {
        if (b_cv[b_in_idx] != ct_data[b_in_idx]) {
          exitg1 = 1;
        } else {
          b_in_idx++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    b_in_idx = 0;
  } else {
    result = false;
    if (patt_idx == 16) {
      b_in_idx = 0;
      do {
        exitg1 = 0;
        if (b_in_idx < 16) {
          if (cv1[b_in_idx] != ct_data[b_in_idx]) {
            exitg1 = 1;
          } else {
            b_in_idx++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      b_in_idx = 1;
    } else {
      result = false;
      if (patt_idx == 13) {
        b_in_idx = 0;
        do {
          exitg1 = 0;
          if (b_in_idx < 13) {
            if (cv2[b_in_idx] != ct_data[b_in_idx]) {
              exitg1 = 1;
            } else {
              b_in_idx++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        b_in_idx = 2;
      } else {
        b_in_idx = -1;
      }
    }
  }
  switch (b_in_idx) {
  case 0:
    memcpy(&thresholds[0], &KE_tables.Average.HeadOnCollision[0],
           8U * sizeof(real_T));
    break;
  case 1:
    memcpy(&thresholds[0], &KE_tables.Average.RearEndCollision[0],
           8U * sizeof(real_T));
    break;
  case 2:
    memcpy(&thresholds[0], &KE_tables.Average.SideCollision[0],
           8U * sizeof(real_T));
    break;
  default:
    memcpy(&thresholds[0], &KE_tables.Average.ObliqueCollision[0],
           8U * sizeof(real_T));
    break;
  }
}

void d_VehicleCollisionSeverity_get_(void)
{
  static real_T dv[8] = {0.0, 4.75, 20.5, 47.5, 4.75, 20.5, 47.5, 0.0};
  static real_T dv1[8] = {0.0, 2.5, 6.0, 39.0, 2.5, 6.0, 39.0, 0.0};
  static real_T dv2[8] = {0.0, 7.0, 35.0, 52.5, 7.0, 35.0, 52.5, 0.0};
  static real_T dv3[8] = {0.0, 10.0, 40.0, 50.0, 6.5, 40.0, 50.0, 0.0};
  static real_T dv4[8] = {0.0, 10.0, 30.0, 40.0, 3.0, 30.0, 40.0, 0.0};
  static real_T dv5[8] = {0.0, 10.0, 50.0, 60.0, 10.0, 50.0, 60.0, 0.0};
  static real_T dv6[8] = {0.0, 10.0, 50.0, 65.0, 10.0, 50.0, 65.0, 0.0};
  static real_T dv7[8] = {0.0, 3.0, 14.0, 28.0, 3.0, 14.0, 28.0, 0.0};
  static real_T dv8[8] = {0.0, 2.0, 8.0, 16.0, 2.0, 8.0, 16.0, 0.0};
  static real_T dv9[8] = {0.0, 4.0, 20.0, 40.0, 4.0, 20.0, 40.0, 0.0};
  dv[7] = rtInf;
  dv1[7] = rtInf;
  dv2[7] = rtInf;
  dv3[7] = rtInf;
  dv4[7] = rtInf;
  dv5[7] = rtInf;
  dv6[7] = rtInf;
  dv7[7] = rtInf;
  dv8[7] = rtInf;
  dv9[7] = rtInf;
  memcpy(&KE_tables.LowerBound.HeadOnCollision[0], &dv9[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.LowerBound.RearEndCollision[0], &dv9[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.LowerBound.SideCollision[0], &dv8[0], 8U * sizeof(real_T));
  memcpy(&KE_tables.LowerBound.ObliqueCollision[0], &dv7[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.HigherBound.HeadOnCollision[0], &dv6[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.HigherBound.RearEndCollision[0], &dv5[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.HigherBound.SideCollision[0], &dv4[0], 8U * sizeof(real_T));
  memcpy(&KE_tables.HigherBound.ObliqueCollision[0], &dv3[0],
         8U * sizeof(real_T));
  memcpy(&KE_tables.Average.HeadOnCollision[0], &dv2[0], 8U * sizeof(real_T));
  memcpy(&KE_tables.Average.RearEndCollision[0], &dv2[0], 8U * sizeof(real_T));
  memcpy(&KE_tables.Average.SideCollision[0], &dv1[0], 8U * sizeof(real_T));
  memcpy(&KE_tables.Average.ObliqueCollision[0], &dv[0], 8U * sizeof(real_T));
}

/* End of code generation (VehicleCollisionSeverity.c) */
