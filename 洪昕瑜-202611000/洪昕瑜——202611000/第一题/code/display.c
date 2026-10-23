#include "display.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

//----------------------------------------------------------------
// 函数简介     刷新当前/待执行方案显示
//----------------------------------------------------------------
void Display_Update(uint8_t current, uint8_t next){
    NEUQ_LCD200_DisplayString(106, 93, "Current:");
    NEUQ_LCD200_Display_int32(202, 93, current, 1);

    NEUQ_LCD200_DisplayString(106, 123, "Next   :");
    NEUQ_LCD200_Display_int32(202, 123, next, 1);
}
