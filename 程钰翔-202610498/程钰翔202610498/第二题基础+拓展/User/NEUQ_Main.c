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
key_type KEY_UP;
key_type KEY_DOWN;
key_type KEY_BACK;
key_type KEY_IN;
extern volatile uint8_t  current_key_id;   //当前的按键编码
#define PAGE_MAIN   0    //主菜单
#define PAGE_NEXT   1    //第二页,其实不太确定是放在头文件合适还是原文件
#define MPU_ATTITUDE_DT   (0.05f)   //姿态解算积分步长(秒)=主循环延时/1000，NEUQ_Delay_ms(50) 对应 0.05s
extern uint8_t current_page;      //初始化当前菜单是主菜单
extern uint8_t current_choice;           //初始化当前选项是0选项，我规定的三个选项是0，1，2
extern uint8_t current_choice_new;
extern uint8_t param_change_choice;       //二级菜单中修改参数，0时无法修改，点击按键in变为1，可以修改
extern float target_coefficient;       //为啥加f，这是滤波系数,初始值定为0.2       如果无法编译就加extern在.h里
extern float gyro_x_filtered;             //x轴滤波后的值
extern float gyro_y_filtered;             //y轴滤波后的值
extern float gyro_z_filtered;             //z轴滤波后的值
extern float gyro_x_real;                 //x轴原始数据
extern float gyro_y_real;                 //y轴原始数据
extern float gyro_z_real;                 //z轴原始数据
extern float acc_x_filtered;             //x轴滤波后的值
extern float acc_y_filtered;             //y轴滤波后的值
extern float acc_z_filtered;             //z轴滤波后的值
extern float acc_x_real;                 //x轴原始数据
extern float acc_y_real;                 //y轴原始数据
extern float acc_z_real;                 //z轴原始数据
extern uint8_t target_send_state;         //传送状态 0为停止发送 1为开始发送
extern uint8_t target_axis;               //0为x轴  1为y轴  2为x轴

// 陀螺仪零偏 gyro_x/y/z_offset 及其标定/跟踪函数已由 NEUQ 库统一提供
//（声明见 NEUQ_IMU_Solve.h），此处无需重复声明。

// **************************** 代码区域 ****************************
void User_Main(void){
  
    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();
    NEUQ_MPU6050_init(); // 初始化MPU6050传感器
    mpu_solve_zero();    // 上电静止标定陀螺仪三轴零偏（约 1.2s，期间须保持静止）
    NEUQ_LCD200_init();
    Menu_Display();/*刷新*/
   NEUQ_KEY_ALL_Init();
    NEUQ_Debug_init();
    NEUQ_Flash_init();
    NEUQ_Flash_read_sector(511,(uint8_t *)&target_coefficient,4);
    mpu_mahony_init();                   // 初始化 Mahony 四元数姿态解算，上电时姿态归零
    NEUQ_PIT_init_ms(PIT_CH1,1);  // 初始化PIT1，周期1毫秒
    

   
    while(1){
        NEUQ_MPU6050_read_data() ;// 读取MPU6050传感器数据，数据保存在 MPU6050_Data_get 结构体中
        MPU_Data_Update();
        //静止时自动跟踪陀螺仪零偏，抑制 yaw 缓慢漂移（NEUQ 库实现）
        mpu_gyro_offset_track(gyro_x_real, gyro_y_real, gyro_z_real, acc_x_real, acc_y_real, acc_z_real);

       //Mahony 四元数姿态解算，使用固定积分步长 MPU_ATTITUDE_DT，以后改延时要改dt=延时（ms）/1000（比如延时50ms，则dt=0.05s）
        mpu_mahony_update(gyro_x_real, gyro_y_real, gyro_z_real, acc_x_real, acc_y_real, acc_z_real, MPU_ATTITUDE_DT);   // Mahony 四元数姿态解算
        mpu_get_attitude();                                           // 由四元数计算 roll/pitch/yaw


       //NEUQ_KEY_ALL_SCAN();//按键扫描已移到 PIT1 的 1ms 中断中
       MENU_Act();

        LowPassFilter_Update();//滤波处理
        Target_Send();
          
        Menu_Display();
        NEUQ_Delay_ms(50);

    }
}