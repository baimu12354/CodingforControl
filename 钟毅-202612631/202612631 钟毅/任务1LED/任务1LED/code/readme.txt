code 文件夹：存放工程依赖的库代码（自定义 .c/.h 也放这里）

HAL_lib            ST 官方底层库：CMSIS Core + STM32F4xx HAL 驱动
NEUQ_lib           NEUQ 开源库
  NEUQ_lib_Driver  片上外设驱动：GPIO/UART/PWM/ADC/Encoder/Timer/PIT/EXTI/IIC/SPI/FIFO/Flash/Delay 等
  NEUQ_lib_Device  外设模块驱动：LCD200 / MPU6050 / OV5640 / OV7725 / OV9281
  NEUQ_lib_APP     应用层算法：PID / 滤波 / IMU 解算
  NEUQ_lib_Debug   调试组件：UART 调试输出 / 断言 / 逐飞接口
  NEUQ_lib_include.h  库总头文件

user 文件夹：存放主函数 NEUQ_Main.c 与中断函数 NEUQ_Interrupts.c
