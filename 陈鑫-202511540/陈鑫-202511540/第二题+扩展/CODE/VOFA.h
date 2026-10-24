#ifndef __VOFA_H
#define __VOFA_H

#include "NEUQ_lib_include.h"

void vofa_send(float raw,float filt);
void vofa_suspend(void);
void vofa_resume(void);

#endif