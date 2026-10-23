/* ===== NEUQ 第三题：直流电机转速闭环 =====
 *  移植自朋友可用的 motor_ctrl 模块，引脚不变：
 *    PWM:  TIM3_CH1_PC6  DIR: PB12  EN: PB13
 *    编码器: TIM4 PB6/PB7   串口: UART_0 (PA9/PA10)
 */
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "motor_ctrl.h"

// **************************** 代码区域 ****************************
void User_Main(void){

    // 初始化默认调试串口（也负责 printf 重定向，保留不动）
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    // 电机闭环控制初始化(串口/编码器/PWM/PID/定时中断)
    MotorCtrl_Init();

    while(1){
        MotorCtrl_Loop();   // 串口解包 + 周期上报(不阻塞电机控制)
    }
}
