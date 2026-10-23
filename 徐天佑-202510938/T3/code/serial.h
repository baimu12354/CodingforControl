#ifndef __SERIAL_H__
#define __SERIAL_H__


extern float td_target ;
int32_t Serial_Read(void);
void Serial_update(void);
void Motor_SendFeedback(void);


#endif