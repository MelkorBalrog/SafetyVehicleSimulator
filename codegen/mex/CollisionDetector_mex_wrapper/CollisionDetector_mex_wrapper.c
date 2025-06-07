/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * CollisionDetector_mex_wrapper.c
 *
 * Code generation for function 'CollisionDetector_mex_wrapper'
 *
 */

/* Include files */
#include "CollisionDetector_mex_wrapper.h"
#include "CollisionDetector.h"
#include "CollisionDetector_mex_wrapper_data.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    12,                              /* lineNo */
    "CollisionDetector_mex_wrapper", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Scripts\\Wrappers\\CollisionDetector_mex_wrapper.m" /* pathName
                                                                     */
};

static emlrtRSInfo b_emlrtRSI = {
    92,                                 /* lineNo */
    "CollisionDetector/checkCollision", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    95,                                 /* lineNo */
    "CollisionDetector/checkCollision", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    98,                                 /* lineNo */
    "CollisionDetector/checkCollision", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    103,                                /* lineNo */
    "CollisionDetector/checkCollision", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    194,                         /* lineNo */
    "CollisionDetector/getAxes", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    196,                         /* lineNo */
    "CollisionDetector/getAxes", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    203,                         /* lineNo */
    "CollisionDetector/getAxes", /* fcnName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    37,       /* lineNo */
    "unique", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo l_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI =
    {
        13,    /* lineNo */
        "any", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025a\\toolbox\\eml\\lib\\matlab\\ops\\any.m" /* pathName
                                                                        */
};

static emlrtBCInfo emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    200,                         /* lineNo */
    49,                          /* colNo */
    "axes_norm",                 /* aName */
    "CollisionDetector/getAxes", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    200,                         /* lineNo */
    23,                          /* colNo */
    "axes_norm",                 /* aName */
    "CollisionDetector/getAxes", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m", /* pName */
    0                                                  /* checkKind */
};

static emlrtECInfo emlrtECI = {
    2,                                  /* nDims */
    103,                                /* lineNo */
    20,                                 /* colNo */
    "CollisionDetector/checkCollision", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    2,                                  /* nDims */
    103,                                /* lineNo */
    36,                                 /* colNo */
    "CollisionDetector/checkCollision", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    -1,                          /* nDims */
    200,                         /* lineNo */
    13,                          /* colNo */
    "CollisionDetector/getAxes", /* fName */
    "C:\\Users\\migue\\codex\\z\\VDSS---Vehicle-Dynamics-Safety-"
    "Simulator\\Source\\Physics\\CollisionDetector.m" /* pName */
};

/* Function Definitions */
boolean_T CollisionDetector_mex_wrapper(const emlrtStack *sp,
                                        const real_T corners1[8],
                                        const real_T corners2[8])
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T proj1_data[32];
  real_T axes_data[16];
  real_T axes_norm_data[16];
  real_T axes_raw[16];
  real_T edges1[8];
  real_T edges2[8];
  real_T p2max_data[8];
  real_T p2min_data[8];
  real_T edges1_tmp;
  real_T edges2_tmp;
  int32_T axes_norm_size[2];
  int32_T axes_size[2];
  int32_T p1max_size[2];
  int32_T p1min_size[2];
  int32_T p2max_size[2];
  int32_T tmp_size[2];
  int32_T x_size[2];
  int32_T b_n;
  int32_T b_trueCount;
  int32_T i;
  int32_T k;
  int32_T n;
  int32_T trueCount;
  int8_T d_tmp_data[8];
  int8_T e_tmp_data[8];
  int8_T tmp_data[8];
  int8_T b_i;
  boolean_T b_tmp_data[8];
  boolean_T c_tmp_data[8];
  boolean_T f_tmp_data[8];
  boolean_T flip_idx_data[8];
  boolean_T x_data[8];
  boolean_T collision;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  /*  CollisionDetector_mex_wrapper - wrapper function for
   * CollisionDetector.checkCollision for MEX generation */
  /*  Usage: */
  /*    collision = CollisionDetector_mex_wrapper(corners1, corners2) */
  /*  Inputs: */
  /*    corners1: 4x2 double matrix */
  /*    corners2: 4x2 double matrix */
  /*  Create detector instance */
  /*  Call method */
  st.site = &emlrtRSI;
  /* /_* */
  /*  * @brief Determines if two rectangles collide using the Separating Axis
   * Theorem (SAT). */
  /*  * */
  /*  * This method checks for collisions between two rectangular vehicles by
   * projecting their corners onto potential separating axes. */
  /*  * If no separating axis is found, a collision is detected. */
  /*  * */
  /*  * @param corners1 A 4x2 matrix containing the (X, Y) coordinates of the
   * first rectangle's corners. */
  /*  * @param corners2 A 4x2 matrix containing the (X, Y) coordinates of the
   * second rectangle's corners. */
  /*  * */
  /*  * @return collision A boolean value indicating if a collision is detected
   * (`true`) or not (`false`). */
  /*  * */
  /*  * @throws Error if the input matrices do not have the correct dimensions.
   */
  /*  checkCollision Determines if two rectangles collide using SAT. */
  /*  */
  /*  Inputs: */
  /*    corners1 - 4x2 matrix containing the (X, Y) coordinates of the first
   * rectangle's corners. */
  /*    corners2 - 4x2 matrix containing the (X, Y) coordinates of the second
   * rectangle's corners. */
  /*  */
  /*  Output: */
  /*    collision - Boolean indicating if a collision is detected (true) or not
   * (false). */
  /*  Validate input dimensions */
  /*  Get the axes (normals) to be tested */
  b_st.site = &b_emlrtRSI;
  /* /_* */
  /*  * @brief Computes the set of unique axes (normals) to test for the
   * Separating Axis Theorem (SAT). */
  /*  * */
  /*  * This method extracts the normals of all edges from both rectangles and
   * removes duplicate axes */
  /*  * to optimize performance during collision detection. */
  /*  * */
  /*  * @param corners1 A 4x2 matrix of the first rectangle's corners. */
  /*  * @param corners2 A 4x2 matrix of the second rectangle's corners. */
  /*  * */
  /*  * @return axes An Nx2 matrix of unit vectors representing the axes to
   * test. */
  /*  _/ */
  /*  getAxes Computes the set of unique axes (normals) to test for SAT. */
  /*  */
  /*  Inputs: */
  /*    corners1 - 4x2 matrix of the first rectangle's corners. */
  /*    corners2 - 4x2 matrix of the second rectangle's corners. */
  /*  */
  /*  Output: */
  /*    axes - Nx2 matrix of unit vectors representing the axes to test. */
  /*  Compute edge normals for both rectangles */
  for (k = 0; k < 2; k++) {
    real_T b_edges1_tmp;
    real_T b_edges2_tmp;
    real_T c_edges1_tmp;
    real_T c_edges2_tmp;
    n = k << 2;
    edges1_tmp = corners1[n + 1];
    b_edges1_tmp = corners1[n];
    edges1[n] = edges1_tmp - b_edges1_tmp;
    edges2_tmp = corners2[n + 1];
    b_edges2_tmp = corners2[n];
    edges2[n] = edges2_tmp - b_edges2_tmp;
    c_edges1_tmp = corners1[n + 2];
    edges1[n + 1] = c_edges1_tmp - edges1_tmp;
    c_edges2_tmp = corners2[n + 2];
    edges2[n + 1] = c_edges2_tmp - edges2_tmp;
    edges1_tmp = corners1[n + 3];
    edges1[n + 2] = edges1_tmp - c_edges1_tmp;
    edges2_tmp = corners2[n + 3];
    edges2[n + 2] = edges2_tmp - c_edges2_tmp;
    edges1[n + 3] = b_edges1_tmp - edges1_tmp;
    edges2[n + 3] = b_edges2_tmp - edges2_tmp;
  }
  /*  Combine and normalize, removing zero-length vectors */
  r = _mm_loadu_pd(&edges1[4]);
  r1 = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&axes_raw[0], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&edges1[0]);
  _mm_storeu_pd(&axes_raw[8], r);
  r = _mm_loadu_pd(&edges2[4]);
  _mm_storeu_pd(&axes_raw[4], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&edges2[0]);
  _mm_storeu_pd(&axes_raw[12], r);
  r = _mm_loadu_pd(&edges1[6]);
  _mm_storeu_pd(&axes_raw[2], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&edges1[2]);
  _mm_storeu_pd(&axes_raw[10], r);
  r = _mm_loadu_pd(&edges2[6]);
  _mm_storeu_pd(&axes_raw[6], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&edges2[2]);
  _mm_storeu_pd(&axes_raw[14], r);
  for (k = 0; k <= 14; k += 2) {
    r = _mm_loadu_pd(&axes_raw[k]);
    _mm_storeu_pd(&axes_data[k], _mm_mul_pd(r, r));
  }
  r = _mm_loadu_pd(&axes_data[0]);
  r1 = _mm_loadu_pd(&axes_data[8]);
  _mm_storeu_pd(&edges1[0], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&axes_data[2]);
  r1 = _mm_loadu_pd(&axes_data[10]);
  _mm_storeu_pd(&edges1[2], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&axes_data[4]);
  r1 = _mm_loadu_pd(&axes_data[12]);
  _mm_storeu_pd(&edges1[4], _mm_add_pd(r, r1));
  r = _mm_loadu_pd(&axes_data[6]);
  r1 = _mm_loadu_pd(&axes_data[14]);
  _mm_storeu_pd(&edges1[6], _mm_add_pd(r, r1));
  c_st.site = &f_emlrtRSI;
  r = _mm_loadu_pd(&edges1[0]);
  _mm_storeu_pd(&edges1[0], _mm_sqrt_pd(r));
  r = _mm_loadu_pd(&edges1[2]);
  _mm_storeu_pd(&edges1[2], _mm_sqrt_pd(r));
  r = _mm_loadu_pd(&edges1[4]);
  _mm_storeu_pd(&edges1[4], _mm_sqrt_pd(r));
  r = _mm_loadu_pd(&edges1[6]);
  _mm_storeu_pd(&edges1[6], _mm_sqrt_pd(r));
  c_st.site = &g_emlrtRSI;
  trueCount = 0;
  n = 0;
  for (k = 0; k < 8; k++) {
    if (edges1[k] > 0.0) {
      trueCount++;
      tmp_data[n] = (int8_T)k;
      n++;
    }
  }
  axes_norm_size[0] = trueCount;
  axes_norm_size[1] = 2;
  for (k = 0; k < 2; k++) {
    for (i = 0; i < trueCount; i++) {
      b_i = tmp_data[i];
      axes_norm_data[i + trueCount * k] =
          axes_raw[b_i + (k << 3)] / edges1[b_i];
    }
  }
  /*  Canonicalize axis directions (ensure unique representation) */
  for (k = 0; k < trueCount; k++) {
    b_tmp_data[k] =
        (muDoubleScalarAbs(axes_norm_data[k]) < 2.2204460492503131E-16);
    c_tmp_data[k] = (axes_norm_data[k + trueCount] < 0.0);
    flip_idx_data[k] = (axes_norm_data[k] < 0.0);
  }
  for (k = 0; k < trueCount; k++) {
    b_tmp_data[k] = (b_tmp_data[k] && c_tmp_data[k]);
  }
  for (k = 0; k < trueCount; k++) {
    flip_idx_data[k] = (flip_idx_data[k] || b_tmp_data[k]);
  }
  b_n = 0;
  for (k = 0; k < trueCount; k++) {
    if (flip_idx_data[k]) {
      b_n++;
    }
  }
  n = 0;
  for (k = 0; k < trueCount; k++) {
    if (flip_idx_data[k]) {
      if (k > trueCount - 1) {
        emlrtDynamicBoundsCheckR2012b(k, 0, trueCount - 1, &b_emlrtBCI, &b_st);
      }
      d_tmp_data[n] = (int8_T)k;
      n++;
    }
  }
  b_trueCount = 0;
  n = 0;
  for (k = 0; k < trueCount; k++) {
    if (flip_idx_data[k]) {
      b_trueCount++;
      e_tmp_data[n] = (int8_T)k;
      n++;
    }
  }
  axes_size[0] = b_trueCount;
  axes_size[1] = 2;
  for (k = 0; k < 2; k++) {
    for (i = 0; i < b_trueCount; i++) {
      b_i = e_tmp_data[i];
      if (b_i > trueCount - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, trueCount - 1, &emlrtBCI, &b_st);
      }
      axes_data[i + b_trueCount * k] = -axes_norm_data[b_i + trueCount * k];
    }
  }
  p2max_size[0] = b_n;
  p2max_size[1] = 2;
  emlrtSubAssignSizeCheckR2012b(&p2max_size[0], 2, &axes_size[0], 2,
                                &c_emlrtECI, &b_st);
  for (k = 0; k < 2; k++) {
    for (i = 0; i < b_n; i++) {
      axes_norm_data[d_tmp_data[i] + trueCount * k] =
          axes_data[i + b_trueCount * k];
    }
  }
  /*  Obtain unique axes (rows) */
  c_st.site = &h_emlrtRSI;
  d_st.site = &k_emlrtRSI;
  unique_rows(&d_st, axes_norm_data, axes_norm_size, axes_data, axes_size);
  /*  Vectorized projection of both rectangles onto each axis */
  b_st.site = &c_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  mtimes(corners1, axes_data, axes_size, proj1_data, axes_norm_size);
  b_n = axes_norm_size[1];
  p1min_size[0] = 1;
  p1min_size[1] = axes_norm_size[1];
  if (axes_norm_size[1] >= 1) {
    for (k = 0; k < b_n; k++) {
      edges1_tmp = proj1_data[4 * k];
      for (i = 0; i < 3; i++) {
        edges2_tmp = proj1_data[(i + 4 * k) + 1];
        if (muDoubleScalarIsNaN(edges2_tmp)) {
          collision = false;
        } else if (muDoubleScalarIsNaN(edges1_tmp)) {
          collision = true;
        } else {
          collision = (edges1_tmp > edges2_tmp);
        }
        if (collision) {
          edges1_tmp = edges2_tmp;
        }
      }
      edges1[k] = edges1_tmp;
    }
  }
  p1max_size[0] = 1;
  p1max_size[1] = (int8_T)axes_norm_size[1];
  if (axes_norm_size[1] >= 1) {
    for (k = 0; k < b_n; k++) {
      edges1_tmp = proj1_data[4 * k];
      for (i = 0; i < 3; i++) {
        edges2_tmp = proj1_data[(i + 4 * k) + 1];
        if (muDoubleScalarIsNaN(edges2_tmp)) {
          collision = false;
        } else if (muDoubleScalarIsNaN(edges1_tmp)) {
          collision = true;
        } else {
          collision = (edges1_tmp < edges2_tmp);
        }
        if (collision) {
          edges1_tmp = edges2_tmp;
        }
      }
      edges2[k] = edges1_tmp;
    }
  }
  b_st.site = &d_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  mtimes(corners2, axes_data, axes_size, proj1_data, axes_norm_size);
  n = axes_norm_size[1];
  axes_norm_size[0] = 1;
  if (axes_norm_size[1] >= 1) {
    for (k = 0; k < n; k++) {
      edges1_tmp = proj1_data[4 * k];
      for (i = 0; i < 3; i++) {
        edges2_tmp = proj1_data[(i + 4 * k) + 1];
        if (muDoubleScalarIsNaN(edges2_tmp)) {
          collision = false;
        } else if (muDoubleScalarIsNaN(edges1_tmp)) {
          collision = true;
        } else {
          collision = (edges1_tmp > edges2_tmp);
        }
        if (collision) {
          edges1_tmp = edges2_tmp;
        }
      }
      p2min_data[k] = edges1_tmp;
    }
  }
  p2max_size[0] = 1;
  p2max_size[1] = (int8_T)axes_norm_size[1];
  if (axes_norm_size[1] >= 1) {
    for (k = 0; k < n; k++) {
      edges1_tmp = proj1_data[4 * k];
      for (i = 0; i < 3; i++) {
        edges2_tmp = proj1_data[(i + 4 * k) + 1];
        if (muDoubleScalarIsNaN(edges2_tmp)) {
          collision = false;
        } else if (muDoubleScalarIsNaN(edges1_tmp)) {
          collision = true;
        } else {
          collision = (edges1_tmp < edges2_tmp);
        }
        if (collision) {
          edges1_tmp = edges2_tmp;
        }
      }
      p2max_data[k] = edges1_tmp;
    }
  }
  /*  Check for any separating axis */
  n = p1max_size[1];
  if ((p1max_size[1] != axes_norm_size[1]) &&
      ((p1max_size[1] != 1) && (axes_norm_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(p1max_size[1], axes_norm_size[1], &emlrtECI,
                                &st);
  }
  b_trueCount = (int8_T)axes_norm_size[1];
  if (((int8_T)axes_norm_size[1] != b_n) &&
      (((int8_T)axes_norm_size[1] != 1) && (b_n != 1))) {
    emlrtDimSizeImpxCheckR2021b((int8_T)axes_norm_size[1], b_n, &b_emlrtECI,
                                &st);
  }
  if (p1max_size[1] == axes_norm_size[1]) {
    x_size[0] = 1;
    x_size[1] = p1max_size[1];
    for (k = 0; k < n; k++) {
      x_data[k] = (edges2[k] < p2min_data[k]);
    }
  } else {
    lt(x_data, x_size, edges2, p1max_size, p2min_data, axes_norm_size);
  }
  if ((int8_T)axes_norm_size[1] == p1min_size[1]) {
    tmp_size[0] = 1;
    tmp_size[1] = (int8_T)axes_norm_size[1];
    for (k = 0; k < b_trueCount; k++) {
      f_tmp_data[k] = (p2max_data[k] < edges1[k]);
    }
  } else {
    lt(f_tmp_data, tmp_size, p2max_data, p2max_size, edges1, p1min_size);
  }
  if ((x_size[1] != tmp_size[1]) && ((x_size[1] != 1) && (tmp_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(x_size[1], tmp_size[1], &emlrtECI, &st);
  }
  b_st.site = &e_emlrtRSI;
  if (x_size[1] == tmp_size[1]) {
    n = x_size[1] - 1;
    for (k = 0; k <= n; k++) {
      x_data[k] = (x_data[k] || f_tmp_data[k]);
    }
  } else {
    b_or(x_data, x_size, f_tmp_data, tmp_size);
  }
  c_st.site = &o_emlrtRSI;
  collision = false;
  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= x_size[1])) {
    if (x_data[n - 1]) {
      collision = true;
      exitg1 = true;
    } else {
      n++;
    }
  }
  if (collision) {
    collision = false;
  } else {
    /*  No separating axis found, collision detected */
    collision = true;
  }
  return collision;
}

emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

/* End of code generation (CollisionDetector_mex_wrapper.c) */
