#ifndef NEUQ_SENSOR_H_
#define NEUQ_SENSOR_H_

extern float sensor_raw[6];
extern float sensor_filt[6];

void NEUQ_Sensor_Init(void);
void NEUQ_Sensor_Task(void);

#endif
