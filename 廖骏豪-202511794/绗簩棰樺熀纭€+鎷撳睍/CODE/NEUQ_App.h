/*
 * NEUQ_App.h
 * 应用层总入口，负责初始化和主循环调度
 *
 * 在 User/NEUQ_Main.c 里的用法：
 *     void User_Main(void)
 *     {
 *         NEUQ_App_Init();        // 只调一次
 *         while(1)
 *         {
 *             NEUQ_App_Loop();    // 反复调，里面做分时调度
 *         }
 *     }
 *
 * 为什么不用定时器中断采样：
 *   库里的I2C读是HAL_I2C_Mem_Read(..., 100)，超时判断靠HAL_GetTick()也就是SysTick。
 *   如果在TIM中断里读传感器，I2C一出错中断就在等一个不前进的tick，
 *   会把同样在等SysTick的主循环一起锁死。
 *   所以这里用主循环+DWT微秒计时来分时，采样周期一样准，还不会有中断死锁。
 */
#ifndef _NEUQ_APP_H_
#define _NEUQ_APP_H_

// 系统初始化，上电调一次
void NEUQ_App_Init(void);

// 主循环体，里面按DWT计时分时执行
//    每   2ms：读MPU6050 -> 六轴滤波 -> 四元数姿态更新 -> 按速率上传
//    每  10ms：按键扫描 + 处理按键事件
//    每 100ms：刷新当前页面
void NEUQ_App_Loop(void);

#endif /* _NEUQ_APP_H_ */
