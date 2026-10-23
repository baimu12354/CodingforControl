#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "../CODE/NEUQ_KEY.h"

// ==================== 中断文件（NEUQ_Interrupts.c）中的全局变量与函数 ====================
extern volatile uint8_t  g_cur_scheme;      // 当前正在执行的方案：0~3（页面显示 1~4）
extern volatile uint8_t  g_pending_scheme;  // 待执行方案：0~3
extern void Led_Start(void);                 // LED 状态机启动（NEUQ_Interrupts.c 中实现）

Key_type key1;   // 定义一个按键对象，塞到 NEUQ_Key_Init()和 Key_Scan()中使用

// 按键按下要执行的动作（回调）：
// 把“待执行方案”往后切一个（1→2→3→4→1），当前周期不受影响，周期边界才生效。
void OnKeyPress(void)
{
    uint8_t next = (uint8_t)(g_pending_scheme + 1U);      //U的含义是unsigned，表示无符号数，避免溢出时出现负数！！！
    if (next >= 4U)
    {
        next = 0U;
    }
    g_pending_scheme = next;
}

// 刷新 Current: / NEXT: 后面的方案数字：
// 只在对应数字变化时清掉数字区重绘
static void Menu_Update(void)
{
    static uint8_t last_cur  = 0xFFU;   // 上次显示值，0xFF 保证首帧必画，保证cur != last_cur必定成立，如果是0，首帧就不会画了因为条件不成立，导致开机时显示current和next后面是白的，所以只要不是0，1，2，3就都可以。
    static uint8_t last_next = 0xFFU;
    uint8_t cur = g_cur_scheme;         // 0~3
    uint8_t next = g_pending_scheme;    // 0~3

    if (cur != last_cur)
    {
        last_cur = cur;                                         //因为是g_cur_scheme赋值给的cur，并且g_cur_scheme是由中断中的函数Led_State_Machine()控制的，所以cur是本周期结束之后才会变值。
        NEUQ_LCD200_ClearRect(110, 10, 60, 24);                  // 只清数字区
        NEUQ_LCD200_Display_int32(110, 10, (int32_t)cur + 1, 1); // 显示 1~4
    }
    if (next != last_next)
    {
        last_next = next;
        NEUQ_LCD200_ClearRect(245, 10, 60, 24);
        NEUQ_LCD200_Display_int32(245, 10, (int32_t)next + 1, 1);
    }
}

// **************************** 代码区域 ****************************
void User_Main(void){

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);
    NEUQ_PIT_init_ms(PIT_CH1,1);  // 初始化PIT1，周期1毫秒
    NEUQ_LCD200_init();
    MENU_Data();
    // 按键初始化：第三个参数 = 按下时执行的动作OnKeyPress
    NEUQ_Key_Init(&key1, PE7, OnKeyPress);
   // NEUQ_Timer_init();
    Led_Start();                 // LED 状态机从当前方案第一个动作开始
  


    while(1){
        if(Key_Scan(&key1)){
            OnKeyPress();   // 扫描按键：按下沿自动调用 OnKeyPress
        } // 扫描按键：按下沿自动调用 OnKeyPress
        Menu_Update();           // 刷新 Current:/NEXT: 数字
    }
}
 