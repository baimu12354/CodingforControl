#include"NEUQ_lib_include.h"

static MENU_ID current_send_ch = MENU_NONE;//当前发送波形的轴
static char buf[64];   //发送缓冲区

//各个发送函数
void WaveSendX_Acc(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.x_accel, acgy_raw.x_accel);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}

void WaveSendY_Acc(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.y_accel, acgy_raw.y_accel);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}

void WaveSendZ_Acc(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.z_accel, acgy_raw.z_accel);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}

void WaveSendX_Gyr(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.x_gyro, acgy_raw.x_gyro);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}

void WaveSendY_Gyr(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.y_gyro, acgy_raw.y_gyro);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}

void WaveSendZ_Gyr(void)
{
    sprintf(buf,"%.3f,%.3f\r\n", acandgy.z_gyro, acgy_raw.z_gyro);
    NEUQ_Uart_write_data(UART_0,(uint8_t *)buf, strlen(buf));
}
//设置发送通道
void SendToVOFA_Channel(MENU_ID select_id)
{
    current_send_ch = select_id;
}

// 停止发送
void WaveSend_Stop(void)
{
    current_send_ch = MENU_NONE;
}

//发送调度，每采样一次调用一次
void WaveSend_Schedule(void)
{
    switch(current_send_ch){
        case MENU_WAVE_SENT_X_ACC: WaveSendX_Acc(); break;
        case MENU_WAVE_SENT_Y_ACC: WaveSendY_Acc(); break;
        case MENU_WAVE_SENT_Z_ACC: WaveSendZ_Acc(); break;
        case MENU_WAVE_SENT_X_GYR: WaveSendX_Gyr(); break;
        case MENU_WAVE_SENT_Y_GYR: WaveSendY_Gyr(); break;
        case MENU_WAVE_SENT_Z_GYR: WaveSendZ_Gyr(); break;
        default: break;
    }
}
void SendToVOFA(void)
{
    if(select == NULL || select->info == NULL)
    {
        WaveSend_Stop();
        return;
    }
    MENU_ID new_ch = select->info->self_id;
    SendToVOFA_Channel(new_ch);
}