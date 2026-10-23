#ifndef __PID_H__
#define __PID_H__
extern float k_forward; 
extern float k_forward2;
extern float v_filter;
typedef union {
    uint8_t Byte[4];
    int32_t IntVal;
} serial_union_t;

typedef struct PID {
    float Kp;
    float Ki;
    float Kd;
    float lasterror;
    float integral;
    float derivative;
    float target;
    float actual;
    float currenterror;
    float output;
    float lastactual;

} PID;

extern PID Speed;

extern volatile uint8_t Serial_RxFlag;
extern volatile uint8_t Serial_RxPacket[16];
extern volatile uint8_t motor_control_flag;

void pid_init(PID *position);
void pid_update(PID *position);
void Serial_ApplyParameter(void);
void Motor_Control(void);
void Motor_SendFeedback(void);
void User_Main(void);


#endif


