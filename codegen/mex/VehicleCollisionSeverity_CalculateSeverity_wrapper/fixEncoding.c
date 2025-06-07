/*
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * fixEncoding.c
 *
 * Code generation for function 'fixEncoding'
 *
 */

/* Include files */
#include "fixEncoding.h"
#include "VehicleCollisionSeverity_CalculateSeverity_wrapper_data.h"
#include "rt_nonfinite.h"
#include "emlrt.h"
#include <string.h>

/* Function Definitions */
void fixEncoding(const emlrtStack *sp, char_T out_data[], int32_T out_size[2])
{
  static const char_T destEncoding[6] = "UTF-8";
  static const char_T str[3] = "%d";
  emlrtStack st;
  int32_T outLength;
  char_T outBuff[12];
  st.prev = sp;
  st.tls = sp->tls;
  outLength =
      emlrtEncoding2Encoding((char_T *)&cv[0], (char_T *)&destEncoding[0], 0, 3,
                             (char_T *)&str[0], &outBuff[0]);
  st.site = &cb_emlrtRSI;
  if (outLength == -1) {
    emlrtErrorWithMessageIdR2018a(&st, &b_emlrtRTEI,
                                  "Coder:toolbox:EncodingInvalidEncoding",
                                  "Coder:toolbox:EncodingInvalidEncoding", 0);
  }
  if (outLength == -2) {
    emlrtErrorWithMessageIdR2018a(
        &st, &c_emlrtRTEI, "Coder:toolbox:EncodingConversionError",
        "Coder:toolbox:EncodingConversionError", 3, 4, 12, "windows-1252");
  }
  if (outLength == -3) {
    emlrtErrorWithMessageIdR2018a(
        &st, &d_emlrtRTEI, "Coder:toolbox:EncodingSizeConsistencyError",
        "Coder:toolbox:EncodingSizeConsistencyError", 3, 4, 12, "windows-1252");
  }
  if (outLength == -4) {
    emlrtErrorWithMessageIdR2018a(
        &st, &e_emlrtRTEI, "EMLRT:runTime:DecodingConversionError",
        "EMLRT:runTime:DecodingConversionError", 3, 4, 12, "windows-1252");
  }
  if (outLength == -5) {
    emlrtErrorWithMessageIdR2018a(
        &st, &f_emlrtRTEI, "EMLRT:runTime:DecodingSizeConsistencyError",
        "EMLRT:runTime:DecodingSizeConsistencyError", 3, 4, 12, "windows-1252");
  }
  if (outLength <= -6) {
    emlrtErrorWithMessageIdR2018a(&st, &g_emlrtRTEI,
                                  "Coder:toolbox:EncodingUnknown",
                                  "Coder:toolbox:EncodingUnknown", 0);
  }
  if (outLength < 1) {
    outLength = 0;
  }
  out_size[0] = 1;
  out_size[1] = outLength;
  if (outLength - 1 >= 0) {
    memcpy(&out_data[0], &outBuff[0], (uint32_T)outLength * sizeof(char_T));
  }
}

/* End of code generation (fixEncoding.c) */
