#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "motor_ctrl.h"

// **************************** PIT中断函数 ****************************
void NEUQ_PIT_1_CALLBACK(void){
    MotorCtrl_PeriodCallback();   // 电机测速 + PID + PWM 更新
}
void NEUQ_PIT_2_CALLBACK(void){

}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
void TS_UART_0_RX_CALLBACK(uint8_t dat){
    MotorCtrl_RxByte(dat);   // 若通信改用 UART_1/2/3, 请把对应回调也改成这句
}

void TS_UART_1_RX_CALLBACK(uint8_t dat){

}

void TS_UART_2_RX_CALLBACK(uint8_t dat){

}

void TS_UART_3_RX_CALLBACK(uint8_t dat){

}
// **************************** 串口中断函数 ****************************


// **************************** 外部中断函数 ****************************
void NEUQ_EXTI0_Callback(void)  {

}

void NEUQ_EXTI1_Callback(void)  {

}

void NEUQ_EXTI2_Callback(void)  {

}

void NEUQ_EXTI3_Callback(void)  {

}

void NEUQ_EXTI4_Callback(void)  {

}

void NEUQ_EXTI5_Callback(void)  {

}

void NEUQ_EXTI6_Callback(void)  {

}

void NEUQ_EXTI7_Callback(void)  {

}

void NEUQ_EXTI8_Callback(void)  {

}

void NEUQ_EXTI9_Callback(void)  {

}

void NEUQ_EXTI10_Callback(void) {

}

void NEUQ_EXTI11_Callback(void) {

}

void NEUQ_EXTI12_Callback(void) {

}

void NEUQ_EXTI13_Callback(void) {

}

void NEUQ_EXTI14_Callback(void) {

}

void NEUQ_EXTI15_Callback(void) {

}
// **************************** 外部中断函数 ****************************
