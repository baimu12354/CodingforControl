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
* 文件名称          NEUQ_Main
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2026-10-16        —           加入速度反馈归零、运行中锁定 PID、死区补偿修复
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

// **************************** 代码区域 ****************************

void Send_To_VOFA(void);

volatile uint8_t is_send_vofa = 0;
volatile uint8_t motor_running = 0;

extern volatile float actual_speed_lv;
extern volatile float actual_speed_rv;

static float Get_Status_Word(void)
{
    float s = 0.0f;
    if (pid_modify_rejected) s += 1.0f;
    if (motor_running)       s += 2.0f;
    return s;
}

void User_Main(void){

    NEUQ_Delay_init();
    NEUQ_FIFO_init(&uart_rx_fifo, uart_rx_buf, sizeof(uart_rx_buf));
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
    Motor_PID_Init();
    NEUQ_MSU_init();
    NEUQ_PIT_init_ms(PIT_CH1, 20);
    NEUQ_PIT_init_ms(PIT_CH2, 20);

    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);

    while(1){
        ReciveAndProcess_Data();

        if (new_packet_flag)
        {
            new_packet_flag = 0;
            switch (packed_data_receive.cmd)
            {
            case MSC_CMD_START:
                if (motor_running) break;

                /* 上位机发 rev/s，直接作为 PID 目标 */
                now_data.g_target_spd = opened_data_receive.target_spd;

                /* 复位增量式 PID 历史误差，避免上次残留导致首周期冲高 */
                stra_speed_wheel_1.error[0] = stra_speed_wheel_1.error[1] = stra_speed_wheel_1.error[2] = 0.0f;
                stra_speed_wheel_2.error[0] = stra_speed_wheel_2.error[1] = stra_speed_wheel_2.error[2] = 0.0f;

                /* 复位 duty 累加器 */
                duty_lv = duty_rv = 0.0f;
                delta_duty_lv = delta_duty_rv = 0.0f;

                /* 速度反馈与编码器归零，VOFA 上从 0 开始 */
                NEUQ_Encoder_Clear(Encoder_TIM3);
                NEUQ_Encoder_Clear(Encoder_TIM4);
                curspeed_lv = curspeed_rv = 0;
                actual_speed_lv = actual_speed_rv = 0.0f;

                motor_running = 1;
                pid_modify_rejected = 0;
                break;

            case MSC_CMD_STOP:
                motor_running = 0;

                NEUQ_PWM_set_duty(TIM2_CH3_PA2, 0);
                NEUQ_PWM_set_duty(TIM8_CH1_PC6, 0);

                duty_lv = duty_rv = 0.0f;
                delta_duty_lv = delta_duty_rv = 0.0f;

                stra_speed_wheel_1.error[0] = stra_speed_wheel_1.error[1] = stra_speed_wheel_1.error[2] = 0.0f;
                stra_speed_wheel_2.error[0] = stra_speed_wheel_2.error[1] = stra_speed_wheel_2.error[2] = 0.0f;

                NEUQ_Encoder_Clear(Encoder_TIM3);
                NEUQ_Encoder_Clear(Encoder_TIM4);
                curspeed_lv = curspeed_rv = 0;
                actual_speed_lv = actual_speed_rv = 0.0f;
                break;

            case MSC_CMD_MODIFY_SPD:
                /* 运行中改速度，单位 rev/s */
                now_data.g_target_spd = opened_data_receive.target_spd;
                break;

            case MSC_CMD_MODIFY_PID:
                /* 运行中锁定：只在停机时允许修改 */
                if (!motor_running)
                {
                    now_data.g_kp = opened_data_receive.kp;
                    now_data.g_ki = opened_data_receive.ki;
                    now_data.g_kd = opened_data_receive.kd;
                    Ipid_parameters_setting(&stra_speed_wheel_1,
                                            now_data.g_kp, now_data.g_ki, now_data.g_kd);
                    Ipid_parameters_setting(&stra_speed_wheel_2,
                                            now_data.g_kp, now_data.g_ki, now_data.g_kd);

                    stra_speed_wheel_1.error[0] = stra_speed_wheel_1.error[1] = stra_speed_wheel_1.error[2] = 0.0f;
                    stra_speed_wheel_2.error[0] = stra_speed_wheel_2.error[1] = stra_speed_wheel_2.error[2] = 0.0f;

                    duty_lv = duty_rv = 0.0f;
                    pid_modify_rejected = 0;
                }
                else
                {
                    pid_modify_rejected = 1;
                }
                break;

            default:
                break;
            }
        }

        if (is_send_vofa)
        {
            is_send_vofa = 0;
            Send_To_VOFA();
        }
    }
}

void Send_To_VOFA(void)
{
    FRAMDEF_VOFA v;
    v.target_spd    = now_data.g_target_spd;   // rev/s
    v.actual_spd_lv = actual_speed_lv;         // rev/s
    v.actual_spd_rv = actual_speed_rv;         // rev/s
    v.kp = now_data.g_kp;
    v.ki = now_data.g_ki;
    v.kd = now_data.g_kd;
    v.status = Get_Status_Word();
    Pack_Data_VOFA(&v);
    pid_modify_rejected = 0;
}