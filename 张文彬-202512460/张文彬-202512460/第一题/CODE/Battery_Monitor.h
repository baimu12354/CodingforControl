#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

/* ADC 初始化、16 次均值采样与电池毫伏换算。 */
#include <stdint.h>
void Battery_Monitor_Init(void);
uint32_t Battery_Monitor_ReadMillivolt(void);

#endif /* BATTERY_MONITOR_H */
