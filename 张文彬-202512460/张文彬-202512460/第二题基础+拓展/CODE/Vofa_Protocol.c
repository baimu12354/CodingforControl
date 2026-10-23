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
********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "Imu_State.h"

void Vofa_Protocol_SendFrame(void)
{
    union
    {
        float value;
        uint8_t bytes[4];
    } number;
    uint8_t frame[12];
    uint8_t i;

    //先将转换过的数据存入number中然后通过union的重解释进行转化位十六进制编码
    number.value = g_imu.raw[g_config.send_axis];
    for (i = 0; i < 4U; i++) frame[i] = number.bytes[i];
    number.value = g_imu.filtered[g_config.send_axis];
    for (i = 0; i < 4U; i++) frame[4U + i] = number.bytes[i];

    /* VOFA+ JustFloat 固定帧尾 00 00 80 7F。 */
    //此处即是第一帧结束的固定符号，在上面注释可见
    frame[8] = 0x00;
    frame[9] = 0x00;
    frame[10] = 0x80;
    frame[11] = 0x7F;
    NEUQ_Uart_write_data(UART_0, frame, sizeof(frame));
}//该函数的目的即是一步一步的将数据写成通信双方可以识别的数据


