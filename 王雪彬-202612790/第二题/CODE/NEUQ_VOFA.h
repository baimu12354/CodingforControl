#ifndef _NEUQ_VOFA_H_
#define _NEUQ_VOFA_H_
#include "NEUQ_lib_include.h"


int NEUQ_VOFA_fmt_float(char *out, float v, int dec);

void NEUQ_VOFA_send_frame(const float *ch, uint8_t ch_count);

#endif
