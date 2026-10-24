#include "NEUQ_lib_include.h"

static uint8_t g_vofa_suspend = 0;



void vofa_suspend(void) { g_vofa_suspend = 1; }
void vofa_resume(void)  { g_vofa_suspend = 0; }

void vofa_send(float raw,float filt){
    if(g_filter_param.send_enable==0){
        return;
    }
    if(g_vofa_suspend){        // ← 新增：保存期间直接丢帧返回
        return;
    }
    printf("%.3f,%.3f\n",raw,filt);
}