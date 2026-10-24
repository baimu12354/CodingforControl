/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Interrupts
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-27        张瑞        加入外部中断回调
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/uart_packet.h"

// **************************** PIT中断函数 ****************************

// ===== 控制环状态（位置式 PID，跨中断保留）=====
static pid_value speed_pid;               // 位置式 PID 结构体：Kp/Ki/Kd + Ek/Ek1/Ek_total/OUT
static uint8_t  pid_ready = 0;            // 一次性初始化标志
static float    pid_integ    = 0.0f;      // 积分贡献（已经乘过 Ki，输出量纲）
static float    pid_err_prev = 0.0f;      // 上一次误差（给微分用）
static float    pid_out_prev = 0.0f;      // 上一拍 PWM 输出（斜坡限幅用）
static float    speed_filtered = 0.0f;    // 低通滤波后的实际转速

#define ENCODER_PPR       11              // 编码器线数
#define GEAR_RATIO        10              // 减速比
#define CONTROL_PERIOD_MS 50              // 控制周期 ms（测速+控制同周期）
#define POS_PID_MAX_I     5000.0f        // 位置式积分限幅（抗积分饱和）
// =============================================

void NEUQ_PIT_1_CALLBACK(void){
    //************ */
    static int cl_counter = 0;
    cl_counter++;
    if(cl_counter >= 1000){
        cl_counter = 0;
    }
    //************定时周期设定 */
    // 每 10ms 刷新速度：测速 + PID 控制
    if(cl_counter % CONTROL_PERIOD_MS == 0){
        // ① 测速：输出轴转速。输出1圈=4×11×10=440脉冲，50ms 内 a 个脉冲 → rpm = a*20*60/440
        int32_t cnt = NEUQ_Encoder_Read(Encoder_TIM3);
        NEUQ_Encoder_Clear(Encoder_TIM3);
        actual_rpm = -(float)cnt * 60000.0f
                   / (4.0f * ENCODER_PPR * GEAR_RATIO * CONTROL_PERIOD_MS);   // 原始测速（未经滤波）
        speed_filtered = speed_filtered * 0.7f + actual_rpm * 0.3f;   // 低通滤波：平滑测速，减小微分项噪声
        actual_rpm = speed_filtered;                                  // 写全局，供回传/画图




        // 一次性初始化 PID（清误差/积分历史）“”“”石山代码“”“”
        if(!pid_ready){
            pid_parameters_setting(&speed_pid, kp, ki, kd);
            pid_ready = 1;
        }




        if(run_flag == 0){
            // 停机：PWM 清零 + 清积分/历史，保证下次启动干净
            NEUQ_Motor_SetDuty(0);
            duty_out = 0;
            speed_pid.Ek       = 0;
            speed_pid.Ek1      = 0;
            speed_pid.Ek_total = 0;
            speed_pid.OUT      = 0;
            pid_integ    = 0.0f;
            pid_err_prev = 0.0f;
            pid_out_prev = 0.0f;
            speed_filtered = 0.0f;
        } else {
            // ② 位置式 PID：积分存"输出量纲"（累加时乘 Ki，限幅 ±10000，抗饱和正确）
            float err = (float)target_rpm - actual_rpm;

            pid_integ += ki * err;
            if (pid_integ >  10000.0f) pid_integ =  10000.0f;
            if (pid_integ < -10000.0f) pid_integ = -10000.0f;

            float out = kp * err + pid_integ + kd * (err - pid_err_prev);
            pid_err_prev = err;

            // ③ 输出限幅
            if(out >  10000.0f) out =  10000.0f;
            if(out < -10000.0f) out = -10000.0f;

            // ④ 斜坡限幅：每拍最多变化 MAX_STEP，防止启动瞬间打满/反向 → 超调振荡
            const float MAX_STEP = 5000.0f;
            if (out - pid_out_prev >  MAX_STEP) out = pid_out_prev + MAX_STEP;
            if (out - pid_out_prev < -MAX_STEP) out = pid_out_prev - MAX_STEP;
            pid_out_prev = out;

            duty_out = (int32_t)out;    // 调试：记录当前占空比

            // ⑤ 更新 PWM
            NEUQ_Motor_SetDuty((int)out);
        }
    }

    // 每 50ms 上报一次（9600 波特率下 10ms 太快，会卡死主循环）
    if(cl_counter % 50 == 0){
        send_flag = 1;
    }
}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************

void TS_UART_0_RX_CALLBACK(uint8_t dat){
    UartPacket_RxByte(dat);   // 收到的字节交给 uart_packet 模块存起来
}

void TS_UART_1_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_2_RX_CALLBACK(uint8_t dat){
    
}

void TS_UART_3_RX_CALLBACK(uint8_t dat){
    
}
// **************************** 串口中断函数 ****************************


// **************************** 外部中断函数 ****************************
void NEUQ_EXTI0_Callback(void)  {

}

void NEUQ_EXTI1_Callback(void)  {

}

void NEUQ_EXTI2_Callback(void)  {

}

void NEUQ_EXTI3_Callback(void)  {

}

void NEUQ_EXTI4_Callback(void)  {

}

void NEUQ_EXTI5_Callback(void)  {

}

void NEUQ_EXTI6_Callback(void)  {

}

void NEUQ_EXTI7_Callback(void)  {

}

void NEUQ_EXTI8_Callback(void)  {

}

void NEUQ_EXTI9_Callback(void)  {

}

void NEUQ_EXTI10_Callback(void) {

}

void NEUQ_EXTI11_Callback(void) {

}

void NEUQ_EXTI12_Callback(void) {

}

void NEUQ_EXTI13_Callback(void) {

}

void NEUQ_EXTI14_Callback(void) {

}

void NEUQ_EXTI15_Callback(void) {

}
// **************************** 外部中断函数 ****************************