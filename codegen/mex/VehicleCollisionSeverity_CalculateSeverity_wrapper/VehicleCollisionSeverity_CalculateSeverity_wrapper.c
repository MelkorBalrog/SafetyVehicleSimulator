/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * VehicleCollisionSeverity_CalculateSeverity_wrapper.c
 *
 * Code generation for function
 * 'VehicleCollisionSeverity_CalculateSeverity_wrapper'
 *
 */

/* Include files */
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper.h"
#include "VehicleCollisionSeverity.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "num2str.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    14,                                                   /* lineNo */
    "VehicleCollisionSeverity_CalculateSeverity_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\VehicleCollisionSeverity_CalculateSever"
    "ity_wrapper.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    181,                                          /* lineNo */
    "VehicleCollisionSeverity/CalculateSeverity", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    183,                                          /* lineNo */
    "VehicleCollisionSeverity/CalculateSeverity", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    186,                                          /* lineNo */
    "VehicleCollisionSeverity/CalculateSeverity", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    187,                                          /* lineNo */
    "VehicleCollisionSeverity/CalculateSeverity", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    327,                                           /* lineNo */
    "VehicleCollisionSeverity/determine_severity", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\VehicleCollisionSeverity.m" /* pathName */
};

/* Function Definitions */
void VehicleCollisionSeverity_CalculateSeverity_wrapper(
    const emlrtStack *sp, real_T *dV1, real_T *dV2, char_T severity1_data[],
    int32_T severity1_size[2], char_T severity2_data[],
    int32_T severity2_size[2])
{
  static const real_T n[3] = {1.0, 0.0, 0.0};
  static const char_T b_cv[17] = {'H', 'e', 'a', 'd', '-', 'O', 'n', ' ', 'C',
                                  'o', 'l', 'l', 'i', 's', 'i', 'o', 'n'};
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T thresholds_vehicle1[8];
  real_T thresholds_vehicle2[8];
  real_T a[3];
  real_T b_a;
  real_T b_scale;
  real_T b_y;
  real_T c_a;
  real_T scale;
  real_T v_rel_normal;
  real_T y;
  int32_T tmp_size[2];
  int32_T exitg1;
  int32_T i;
  char_T tmp_data[25];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  /*  VehicleCollisionSeverity_CalculateSeverity_wrapper - wrapper to call
   * VehicleCollisionSeverity.CalculateSeverity for MEX generation */
  /*  Simple placeholder initialization */
  /*         %% Constructor */
  /*  Constructor to initialize the VehicleCollisionSeverity object */
  /*  */
  /*  Inputs: */
  /*    m1, v1, r1 - Vehicle 1 parameters */
  /*    m2, v2, r2 - Vehicle 2 parameters */
  /*    e, n - Collision parameters */
  /*  Precompute inverse masses for performance */
  /*  Initialize CollisionType for both vehicles */
  /*  Initialize final velocities to initial velocities */
  /*         %% Perform Collision */
  /* /_* */
  /*  * @brief Performs the collision calculations to determine final
   * velocities. */
  /*  * */
  /*  * This method calculates the final velocities of both vehicles
   * post-collision based on */
  /*  * the conservation of linear momentum and the coefficient of restitution.
   */
  /*  * */
  /*  * @return obj The updated VehicleCollisionSeverity object with final
   * velocities. */
  /*  _/ */
  /*  Calculate relative velocity along the normal direction */
  a[0] = -20.0;
  a[1] = 0.0;
  a[2] = 0.0;
  n_t = (ptrdiff_t)3;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  v_rel_normal = ddot(&n_t, &a[0], &incx_t, (real_T *)&n[0], &incy_t);
  /*  If relative velocity is positive, vehicles are moving apart; no collision
   */
  /*  if v_rel_normal > 0 */
  /*      fprintf('Vehicles are moving apart. No collision response
   * applied.\n'); */
  /*      obj.v1_final = obj.v1; */
  /*      obj.v2_final = obj.v2; */
  /*      return; */
  /*  end */
  /*  Calculate impulse scalar using precomputed inverse masses */
  v_rel_normal = -1.3 * v_rel_normal / 0.0016666666666666666;
  /*  Update velocities using inverse masses */
  b_a = v_rel_normal * 0.001;
  c_a = v_rel_normal * 0.00066666666666666664;
  st.site = &emlrtRSI;
  /*         %% Calculate Severity */
  /* /_* */
  /*  * @brief Calculates the severity of the collision for each vehicle based
   * on delta-V thresholds. */
  /*  * */
  /*  * This method computes the change in velocity (delta-V) for both vehicles
   * resulting from the collision */
  /*  * and determines their respective severity ratings based on predefined
   * thresholds. */
  /*  * */
  /*  * @return deltaV_vehicle1 Change in velocity for Vehicle 1 (kph). */
  /*  * @return deltaV_vehicle2 Change in velocity for Vehicle 2 (kph). */
  /*  * @return Vehicle1Severity Severity rating for Vehicle 1 ('S0', 'S1',
   * etc.). */
  /*  * @return Vehicle2Severity Severity rating for Vehicle 2 ('S0', 'S1',
   * etc.). */
  /*  _/ */
  /*  Calculate delta-V for both vehicles */
  scale = 3.3121686421112381E-170;
  /*  Convert from m/s to kph */
  b_scale = 3.3121686421112381E-170;
  v_rel_normal = muDoubleScalarAbs((b_a + 10.0) - 10.0);
  if (v_rel_normal > 3.3121686421112381E-170) {
    y = 1.0;
    scale = v_rel_normal;
  } else {
    v_rel_normal /= 3.3121686421112381E-170;
    y = v_rel_normal * v_rel_normal;
  }
  v_rel_normal = muDoubleScalarAbs((-10.0 - c_a) - -10.0);
  if (v_rel_normal > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = v_rel_normal;
  } else {
    v_rel_normal /= 3.3121686421112381E-170;
    b_y = v_rel_normal * v_rel_normal;
  }
  v_rel_normal = b_a * 0.0 / scale;
  b_a = v_rel_normal * v_rel_normal;
  y += b_a;
  v_rel_normal = (0.0 - c_a * 0.0) / b_scale;
  v_rel_normal *= v_rel_normal;
  b_y += v_rel_normal;
  y += b_a;
  b_y += v_rel_normal;
  y = scale * muDoubleScalarSqrt(y);
  b_a = y * 3.6;
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  v_rel_normal = b_y * 3.6;
  /*  Convert from m/s to kph */
  /*  Determine collision thresholds for Vehicle 1 */
  b_st.site = &b_emlrtRSI;
  c_VehicleCollisionSeverity_get_(&b_st, b_cv, thresholds_vehicle1);
  /*  Determine collision thresholds for Vehicle 2 */
  b_st.site = &c_emlrtRSI;
  c_VehicleCollisionSeverity_get_(&b_st, b_cv, thresholds_vehicle2);
  /*  Determine severity ratings */
  b_st.site = &d_emlrtRSI;
  /* /_* */
  /*  * @brief Determines the severity rating based on delta-V and collision
   * thresholds. */
  /*  * */
  /*  * Compares the delta-V value against the provided thresholds to assign a
   * severity rating. */
  /*  * */
  /*  * @param delta_v Delta-V value in kph. */
  /*  * @param collision_thresholds Nx2 matrix of [min, max] delta-V values in
   * kph. */
  /*  * */
  /*  * @return severity Severity rating as 'S0', 'S1', etc. */
  /*  _/ */
  /*  Determine severity rating based on delta-V and collision thresholds */
  /*  */
  /*  Parameters: */
  /*    delta_v - Delta-V in kph */
  /*    collision_thresholds - Thresholds for the collision type */
  /*  */
  /*  Returns: */
  /*    severity - Severity rating as 'S0', 'S1', etc. */
  severity1_size[0] = 1;
  severity1_size[1] = 2;
  severity1_data[0] = 'S';
  severity1_data[1] = '0';
  /*  Default severity */
  i = 0;
  do {
    exitg1 = 0;
    if (i < 4) {
      if ((b_a > thresholds_vehicle1[i]) &&
          (b_a <= thresholds_vehicle1[i + 4])) {
        c_st.site = &i_emlrtRSI;
        num2str(&c_st, ((real_T)i + 1.0) - 1.0, tmp_data, tmp_size);
        severity1_size[0] = 1;
        severity1_size[1] = tmp_size[1] + 1;
        severity1_data[0] = 'S';
        i = tmp_size[1];
        if (i - 1 >= 0) {
          memcpy(&severity1_data[1], &tmp_data[0],
                 (uint32_T)i * sizeof(char_T));
        }
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      if (b_a > 0.0) {
        /*  If delta_v exceeds all thresholds, assign the highest severity */
        severity1_size[0] = 1;
        severity1_size[1] = 2;
        severity1_data[0] = 'S';
        severity1_data[1] = '3';
      }
      exitg1 = 1;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  } while (exitg1 == 0);
  b_st.site = &e_emlrtRSI;
  /* /_* */
  /*  * @brief Determines the severity rating based on delta-V and collision
   * thresholds. */
  /*  * */
  /*  * Compares the delta-V value against the provided thresholds to assign a
   * severity rating. */
  /*  * */
  /*  * @param delta_v Delta-V value in kph. */
  /*  * @param collision_thresholds Nx2 matrix of [min, max] delta-V values in
   * kph. */
  /*  * */
  /*  * @return severity Severity rating as 'S0', 'S1', etc. */
  /*  _/ */
  /*  Determine severity rating based on delta-V and collision thresholds */
  /*  */
  /*  Parameters: */
  /*    delta_v - Delta-V in kph */
  /*    collision_thresholds - Thresholds for the collision type */
  /*  */
  /*  Returns: */
  /*    severity - Severity rating as 'S0', 'S1', etc. */
  severity2_size[0] = 1;
  severity2_size[1] = 2;
  severity2_data[0] = 'S';
  severity2_data[1] = '0';
  /*  Default severity */
  i = 0;
  do {
    exitg1 = 0;
    if (i < 4) {
      if ((v_rel_normal > thresholds_vehicle2[i]) &&
          (v_rel_normal <= thresholds_vehicle2[i + 4])) {
        c_st.site = &i_emlrtRSI;
        num2str(&c_st, ((real_T)i + 1.0) - 1.0, tmp_data, tmp_size);
        severity2_size[0] = 1;
        severity2_size[1] = tmp_size[1] + 1;
        severity2_data[0] = 'S';
        i = tmp_size[1];
        if (i - 1 >= 0) {
          memcpy(&severity2_data[1], &tmp_data[0],
                 (uint32_T)i * sizeof(char_T));
        }
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      if (v_rel_normal > 0.0) {
        /*  If delta_v exceeds all thresholds, assign the highest severity */
        severity2_size[0] = 1;
        severity2_size[1] = 2;
        severity2_data[0] = 'S';
        severity2_data[1] = '3';
      }
      exitg1 = 1;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  } while (exitg1 == 0);
  *dV1 = b_a;
  *dV2 = v_rel_normal;
}

/* End of code generation (VehicleCollisionSeverity_CalculateSeverity_wrapper.c)
 */
