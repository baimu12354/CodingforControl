#ifndef NEUQ_WAVESEND_H_
#define NEUQ_WAVESEND_H_

void WaveSendX_Acc(void);
void WaveSendY_Acc(void);
void WaveSendZ_Acc(void);
void WaveSendX_Gyr(void);
void WaveSendY_Gyr(void);
void WaveSendZ_Gyr(void);
//void WaveSend_Err(void);发送失败反馈，可加可不加

void SendToVOFA_Channel(MENU_ID select_id);//设置发送通道
void WaveSend_Schedule(void);// 发送调度函数，放采样里调用
void WaveSend_Stop(void);// 停止发送
void SendToVOFA(void);//对外调用的发送函数接口，只置标志位
#endif