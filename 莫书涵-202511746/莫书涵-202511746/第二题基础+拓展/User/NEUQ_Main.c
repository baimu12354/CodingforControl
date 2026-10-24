#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

// **************************** 代码区域 ****************************
void User_Main(void){
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_Timer_init();
    //按键及扫描按键的定时器初始化
    NEUQ_Key_Init();
    NEUQ_PIT_init_ms(PIT_CH1,10);
    //菜单初始化
    NEUQ_Menu_Init();
    //六轴数据处理初始化
    NEUQ_SIX_AXIS_DATA_Init();
    NEUQ_PIT_init_ms(PIT_CH2,10);
    while(1){
        NEUQ_Menu_Task();
    }
}
