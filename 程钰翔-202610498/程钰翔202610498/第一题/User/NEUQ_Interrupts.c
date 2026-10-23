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

volatile uint32_t g_tick           = 0;   // 1ms 系统节拍：中断累加，主循环读取
volatile uint8_t  g_cur_scheme     = 0;   // 当前正在执行的方案：0~3（页面显示 1~4）
volatile uint8_t  g_pending_scheme = 0;   // 待执行方案：0~3，周期边界才生效

#define LED_PIN   PA8                    // LED 输出引脚（任务书指定 PA8）
#define LED_ON    GPIO_High              // “亮”的有效电平

// 动作表：每套方案是一个动作序列，level=1 亮 / 0 灭
// 四套方案完整周期均为 2000ms；ms=0 表示该方案动作已播完（等周期边界从头播放）。
typedef struct
{
    uint8_t  level;
    uint16_t ms;

} Led_Action;

static const Led_Action g_scheme_table[4][4] = {            //定义数组有4行4列，4套方案，每套方案最多4个动作
    { {1, 1000}, {0, 1000}, {0, 0}, {0, 0} },               // 方案1：亮1000 → 灭1000
    { {1,  200}, {0, 1800}, {0, 0}, {0, 0} },               // 方案2：亮200  → 灭1800
    { {1,  200}, {0, 200},  {1, 200}, {0, 1400} },          // 方案3：亮200→灭200→亮200→灭1400
    { {1,  200}, {0, 200},  {1, 600}, {0, 1000} },          // 方案4：亮200→灭200→亮600→灭1000
};

static uint32_t g_cycle_tick = 0;   // 当前周期起始节拍
static uint32_t g_act_tick   = 0;   // 当前动作起始节拍
static uint8_t  g_act_id    = 0;   // 当前动作在方案内的序号

// 写 LED 电平：level=1 亮，0 灭（有效电平由 LED_ON 宏定义决定）
static void Led_Set(uint8_t level)
{
    if (level)
    {
        NEUQ_GPIO_write(LED_PIN, LED_ON);
    }
    else
    {
        NEUQ_GPIO_write(LED_PIN, (LED_ON == GPIO_Low) ? GPIO_High : GPIO_Low);//便于以后如果想要把LED_ON改成低电平也可以直接使用
    }
}

// 上电/复位：从当前方案第一个动作开始循环（由 User_Main 调用）
void Led_Start(void)
{
    g_cycle_tick = 0;
    g_act_id    = 0;
    g_act_tick   = 0;
    Led_Set(g_scheme_table[g_cur_scheme][0].level);//数组的【】里面是行号，列号，也解释了为啥规定初始化时用0而不是1，因为0是数组的第1行更方便。
}

// LED 时序状态机：每 1ms 由 PIT 中断调用一次
// 周期严格 2000ms：周期边界用“起始节拍 + 2000”累加对齐，不产生累计误差
// 按键只改待执行方案；周期边界若待执行 != 当前，才切换方案并从头播放
static void Led_State_Machine(void)
{
    const Led_Action *seq = g_scheme_table[g_cur_scheme];

    // 1) 周期边界：一个完整周期（2000ms）结束，开启下一周期
    if ((uint32_t)(g_tick - g_cycle_tick) >= 2000U)
    {
        g_cycle_tick += 2000U;               // 累加而非取当前值，周期不漂移，意思是防止直接赋值g_cycle_tick=g_tick导致累计误差
        if (g_pending_scheme != g_cur_scheme)  //也解释了为啥只有在结束了这个周期才会切换方案。
        {
            g_cur_scheme = g_pending_scheme; // 新方案在下一周期起点生效
            seq = g_scheme_table[g_cur_scheme];//便于下面的Led_Set(seq[0].level);直接使用
        }
        g_act_id  = 0;                      // 从头播放该方案
        g_act_tick = g_tick;
        Led_Set(seq[0].level);              //就是让LED在current方案中执行第一个动作的电平
        return;
    }

    // 2) 动作边界：当前动作持续时间到点，切到下一个动作
    const Led_Action *act = &seq[g_act_id];  //取g_scheme_table[g_cur_scheme][g_act_id]当前方案的当前地址给到act。
    if ((act->ms != 0U) && ((uint32_t)(g_tick - g_act_tick) >= act->ms))//如果当前动作的持续时间不为0，并且当前节拍减去动作起始节拍大于等于动作持续时间，就说明该动作已经完成。
    {
        g_act_id++;
        g_act_tick = g_tick;
        if (g_act_id < 4U)            // 防御：动作播完后保持当前电平，等周期边界从头播放
        {                             //如果不做防御，g_act_id会增加到4，可是4就超过了数组的范围，导致访问越界，可能会出现不可预料的错误
            act = &seq[g_act_id];     //因为周期边界和动作边界是并行运行的，会在周期边界条件成立时同时成立动作条件导致动作到4.
            if (act->ms != 0U)
            {
                Led_Set(act->level);  // 该动作有效，设置电平
            }
        }
    }
}


// **************************** PIT中断函数 ****************************
void NEUQ_PIT_1_CALLBACK(void){

    g_tick++;               // 1ms 节拍
    Led_State_Machine();    // LED 时序独立运行，主循环怎么刷新都不影响

    
    

}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){

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