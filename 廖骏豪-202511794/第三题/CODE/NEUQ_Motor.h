/*********************************************************************************************************************
* 第三题  串口调参与电机闭环控制
*
* 文件名称          NEUQ_Motor
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38 / EIDE(AC5)
* 适用平台          STM32F407VE
*
* 说明：电机驱动层（PWM + 方向），支持三种常见驱动板接线方式，由 NEUQ_Config.h 的
*       NEUQ_MOTOR_MODE 选择：
*         PH_EN   单 PWM(EN) + 单方向脚(PH)  —— 本考场驱动板 DRV8701E 用的就是这个
*         PWM_DIR 单 PWM + 双方向脚(AIN1/AIN2) —— TB6612 / L298N
*         DUAL_PWM 双路 PWM(IN1/IN2)
*       所有函数都在控制中断里调用，内部只有寄存器写，耗时很短。
********************************************************************************************************************/
#ifndef _NEUQ_MOTOR_H_
#define _NEUQ_MOTOR_H_

/*  函数简介     电机驱动初始化（配置 PWM 与方向引脚）
 *  使用示例     NEUQ_Motor_Init();
 *  备注         初始化后输出为 0，电机不转，等待 START 命令                        */
void NEUQ_Motor_Init(void);

/*  函数简介     设置电机输出
 *  参数说明     out    -10000 ~ +10000，正数正转、负数反转、0 为输出关断（滑行）
 *  使用示例     NEUQ_Motor_SetOutput(3000.0f);
 *  备注         函数内部会再次限幅，并叠加 NEUQ_MOTOR_DIR_SIGN 方向符号             */
void NEUQ_Motor_SetOutput(float out);

/*  函数简介     电机停止（按 NEUQ_STOP_BRAKE 选择刹车或滑行）
 *  使用示例     NEUQ_Motor_Stop();
 *  备注         只有"停机"时才调用；闭环运行中输出为 0 用的是 SetOutput(0)，不会刹车 */
void NEUQ_Motor_Stop(void);

/*  函数简介     读取最近一次实际施加给驱动的输出值（限幅后的）
 *  返回参数     -10000 ~ +10000
 *  使用示例     float out = NEUQ_Motor_GetOutput();                                */
float NEUQ_Motor_GetOutput(void);

#endif /* _NEUQ_MOTOR_H_ */
