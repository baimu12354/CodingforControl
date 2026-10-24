/*********************************************************************************************************************
* 文件名称   upload
* 功能说明   VOFA+ 波形上传实现：读 g_cfg 判断发送开关与选轴，发送“原始值,滤波值”
*            只在发送开关打开时发送；任意时刻只发选中那一轴，不夹带其他轴。
*            原始值与滤波值取自同一次采样（同一帧数据）。
*********************************************************************************************************************/
#include "upload.h"
#include "config.h"

void VOFA_Upload(void)
{
    int16_t raw;
    float   filt;
    int     axis;
    char    buf[32];

    if (g_cfg.send_enable == 0) return;          // 关闭上传 -> 不发送（采集/滤波照常进行）

    axis = g_cfg.selected_axis;                  // 0~5
    if (axis < 0 || axis > 5) return;

    // 同一次采样：原始值与滤波值取自同一帧
    if (axis < 3) { raw = g_acc[axis];       filt = g_acc_f[axis]; }
    else          { raw = g_gyro[axis - 3];  filt = g_gyro_f[axis - 3]; }

    // FireWater协议：两通道逗号分隔、\n结尾；都发原始计数 -> 同单位同纵轴尺度
    sprintf(buf, "%d,%d\n", raw, (int)(filt + 0.5f));
    NEUQ_Uart_write_string(UART_0, (uint8_t *)buf);   // USART1(PA9) -> VOFA+
}
