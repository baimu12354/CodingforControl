/* motor_ctrl — 速度闭环控制: 斜坡整形 + PID 调度 + 1ms 周期 */

#include "motor_ctrl.h"     /* app_common.h → 库 + config.h */
#include "motor_hw.h"
#include "motor_speed.h"
#include "motor_pid.h"
#include "motor_proto.h"
#include "motor_report.h"

/* 共享变量定义(声明见 app_common.h) */
volatile uint8_t running = 0;
volatile float   target_rpm = 0.0f;
volatile float   actual_rpm = 0.0f;
PID_t            pid;

/* 私有状态 */
static volatile float target_ref = 0.0f;    // 斜坡整形后的目标

/* 初始化: 硬件/测速/通信 + PID 参数 + 1ms 控制中断 */
void MotorCtrl_Init(void)
{
    Motor_HW_Init();    // 方向/使能/PWM/编码器/ADC
    Speed_Init();       // 编码器基准计数
    Proto_Init();       // 通信串口 + 接收中断 + 接收 FIFO

    // PID 初始参数
    pid.Kp = 6.8f; pid.Ki = 0.012f; pid.Kd = 0.011f;  // 用户指定初始值
    PID_Reset(&pid);

    running = 0; target_rpm = 0; actual_rpm = 0;

    // 控制周期中断(1ms)
    NEUQ_PIT_init_ms(PIT_CH1, CTRL_PERIOD_MS);
}

/* 1ms 控制周期: 测速 → 斜坡整形 → PID → PWM */
void MotorCtrl_PeriodCallback(void)
{
    // 1) 编码器计数差入环形缓冲
    Speed_Accum1ms();

    // 2) 测速更新(内部按 SPEED_PERIOD_MS 节流) + 低通滤波
    actual_rpm = Speed_Calc(target_rpm, actual_rpm);

    // 3) 目标斜坡整形(非PID): target_ref 以 RAMP_RATE 逼近 target_rpm, 让响应曲线平滑贴合、少超调
    if (running)
    {
        if (target_ref < target_rpm) {
            target_ref += RAMP_RATE_UP;              // 加速: 快斜坡
            if (target_ref > target_rpm) target_ref = target_rpm;
        } else if (target_ref > target_rpm) {
            target_ref -= RAMP_RATE_DOWN;            // 减速: 慢斜坡, 防反向刹车过猛
            if (target_ref < target_rpm) target_ref = target_rpm;
        }
    } else {
        target_ref = actual_rpm;                     // 停止时跟随实际转速, 启动时从当前转速平滑爬
    }

    // 4) 固定周期 PID 计算 + PWM 更新
    if (running)
    {
        Motor_HW_SetPwm(PID_Compute(&pid, target_ref, actual_rpm));
    } else {
        Motor_HW_SetPwm(0);
        PID_Reset(&pid);
    }
}

/* 串口接收字节入口(转发给协议解析) */
void MotorCtrl_RxByte(uint8_t dat)
{
    Proto_RxByte(dat);
}

/* 主循环: 帧解析 + 周期上报 + 电压采样 */
void MotorCtrl_Loop(void)
{
    // 1) 解析下行帧
    Proto_Poll();
    // 2) 周期上报
    Report_Tick();
    // 3) 供电电压采样(主循环里做, 不阻塞控制中断)
    Motor_HW_SampleVoltage();
}
