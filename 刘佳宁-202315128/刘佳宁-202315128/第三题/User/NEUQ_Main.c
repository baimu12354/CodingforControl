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
#include <math.h>   // fabs 取绝对值用

// NEUQ 库里实现了、但头文件漏了声明的函数，补一个声明（串口接收中断使能）
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

// ================= 串口协议常量 =================
#define FRAME_HEAD1        0xAA   // 帧头第1字节
#define FRAME_HEAD2        0x55   // 帧头第2字节
#define CMD_START          0x01   // 启动
#define CMD_STOP           0x02   // 停止
#define CMD_SET_SPEED      0x03   // 设置目标速度(一个float)
#define CMD_SET_PID        0x04   // 设置PID(三个float: Kp Ki Kd)
#define CMD_SET_KP         0x05   // 单独设置Kp(一个float)
#define CMD_SET_KI         0x06   // 单独设置Ki(一个float)
#define CMD_SET_KD         0x07   // 单独设置Kd(一个float)
#define FRAME_MAX_PAYLOAD  16     // 数据最多16字节

// ================= 变量（你原来的变量都保留） =================
float target_spd = 0.0f;    //目标速度
float actual_spd = 0.0f;    //真实速度
volatile uint8_t PIT1_flag = 0;
int16_t lv = 0;
int16_t rv = 0;
float Kp = 100.0f;         // PID 比例（串口可在线改）
float pwm_out = 0.0f;
#define BUF_MAX 64
uint8_t receive_box[BUF_MAX];
uint8_t box_index = 0;
uint8_t motor_enable = 0;

// ---- 新增变量 ----
float Ki = 0.0f;            // PID 积分
float Kd = 0.0f;            // PID 微分
float FF = 10.0f;           // 前馈斜率：油门 ≈ 78 × 速度（串口 "ff78" 在线改）
float FF_bias = 4458.0f;    // 前馈起步油：克服电机死区/摩擦（串口 "fb4458" 在线改）
float pid_i_l = 0.0f, pid_last_l = 0.0f;   // 左轮：积分累计、上次误差
float pid_i_r = 0.0f, pid_last_r = 0.0f;   // 右轮：积分累计、上次误差
volatile uint8_t box_head = 0;   // 串口中断写到的位置
uint8_t tx_count = 0;             // 用来每20ms回传一次的计数
volatile uint32_t rx_cnt = 0;     // 收到的字节总数（诊断用）
volatile uint8_t last_rx_byte = 0;   // 最后收到的1个字节（诊断用）

// ---- 帧解析用到的变量 ----
uint8_t rx_state = 0;                 // 解析状态(0~5)
uint8_t frame_cmd = 0;                // 命令
uint8_t frame_len = 0;                // 数据长度
uint8_t frame_idx = 0;                // 已经收到几个数据
uint8_t frame_chk = 0;                // 校验和累加
uint8_t frame_data[FRAME_MAX_PAYLOAD];   // 数据临时存放

// ---- VOFA+ 滑块/文本命令用的变量 ----
char ascii_buf[16];          // 存文本命令（如 "30"、"start"）
uint8_t ascii_idx = 0;       // 已经存了几个字符
uint8_t ascii_idle = 0;      // 文本命令空闲计时（10ms单位），超时自动执行

// ================= 小工具函数 =================

// 4个字节 -> 一个float（用共用体，让字节和float共用同一块内存）
float BytesToFloat(uint8_t *b){
    union { uint8_t b[4]; float f; } u;
    u.b[0]=b[0]; u.b[1]=b[1]; u.b[2]=b[2]; u.b[3]=b[3];
    return u.f;
}

// 一个float -> 4个字节
void FloatToBytes(float v, uint8_t *b){
    union { uint8_t b[4]; float f; } u;
    u.f = v;
    b[0]=u.b[0]; b[1]=u.b[1]; b[2]=u.b[2]; b[3]=u.b[3];
}

// 串口接收中断回调：把收到的1个字节放进缓冲区（中断里只做这一件事）
void UART_RX_Push(uint8_t dat){
    rx_cnt++;                        // 每收到1个字节就+1（诊断用）
    last_rx_byte = dat;              // 记录最后收到的字节（诊断用）
    uint8_t next = box_head + 1;
    if(next >= BUF_MAX) next = 0;    // 转圈：到末尾了回到开头
    if(next != box_index){           // 缓冲没满才存，满了就丢这个字节
        receive_box[box_head] = dat;
        box_head = next;
    }
}

// 收到一条完整帧后，按命令执行
void Apply_Frame(void){
    switch(frame_cmd){
        case CMD_START:
            motor_enable = 1;                       // 启动
            break;
        case CMD_STOP:
            motor_enable = 0;                       // 停止
            target_spd = 0.0f;                      // 目标也归0，波形跟着停下
            break;
        case CMD_SET_SPEED:
            if(frame_len >= 4){
                target_spd = BytesToFloat(&frame_data[0]);   // 设置目标速度
                if(target_spd >  1000.0f) target_spd =  1000.0f;   // 限幅到 ±1000
                if(target_spd < -1000.0f) target_spd = -1000.0f;
            }
            break;
        case CMD_SET_PID:
            if(frame_len >= 12){
                Kp = BytesToFloat(&frame_data[0]);           // 设置 PID
                Ki = BytesToFloat(&frame_data[4]);
                Kd = BytesToFloat(&frame_data[8]);
                pid_i_l = pid_i_r = 0.0f;                    // 换了参数，积分清零重来
                pid_last_l = pid_last_r = 0.0f;
            }
            break;
        case CMD_SET_KP:
            if(frame_len >= 4){
                Kp = BytesToFloat(&frame_data[0]);           // 单独设置 Kp
                pid_i_l = pid_i_r = 0.0f;                    // 积分清零重来
                pid_last_l = pid_last_r = 0.0f;
            }
            break;
        case CMD_SET_KI:
            if(frame_len >= 4){
                Ki = BytesToFloat(&frame_data[0]);           // 单独设置 Ki
                pid_i_l = pid_i_r = 0.0f;                    // 积分清零
            }
            break;
        case CMD_SET_KD:
            if(frame_len >= 4){
                Kd = BytesToFloat(&frame_data[0]);           // 单独设置 Kd
                pid_last_l = pid_last_r = 0.0f;              // 上次误差清零
            }
            break;
    }
}

// 逐字节解析。帧格式: [0xAA][0x55][CMD][LEN][DATA...]
// 去掉了校验字节：这样 VOFA+ 滑块用 Hex 模式 + %% 占位符就能直接发（值自动变成4字节float小端）
void Parse_Frame_Byte(uint8_t dat){
    switch(rx_state){
        case 0:                                // 等第1个帧头 0xAA
            if(dat == FRAME_HEAD1) rx_state = 1;
            break;
        case 1:                                // 等第2个帧头 0x55
            if(dat == FRAME_HEAD2) rx_state = 2;
            else if(dat != FRAME_HEAD1) rx_state = 0;   // 不是0x55也不是0xAA，从头等
            break;
        case 2:                                // 读命令
            frame_cmd = dat;
            rx_state = 3;
            break;
        case 3:                                // 读数据长度
            if(dat > FRAME_MAX_PAYLOAD){ rx_state = 0; break; }   // 长度非法，丢弃
            frame_len = dat;
            frame_idx = 0;
            if(frame_len == 0){ Apply_Frame(); rx_state = 0; }   // 没数据，立刻执行
            else rx_state = 4;
            break;
        case 4:                                // 收数据
            frame_data[frame_idx++] = dat;
            if(frame_idx >= frame_len){ Apply_Frame(); rx_state = 0; }   // 收够就执行
            break;
    }
}

// 把字符串里的数字转成float（支持 "-30.5" 这种，其它字符直接忽略）
float Parse_Number(char *s){
    float v = 0.0f;      // 整数部分
    float f = 0.0f;      // 小数部分
    float wei = 0.1f;    // 小数位权值
    uint8_t is_dec = 0;  // 是否已经进入小数部分
    uint8_t is_neg = 0;  // 是否是负数
    uint8_t i = 0;
    if(s[0] == '-'){ is_neg = 1; i = 1; }
    for(; s[i] != 0; i++){
        if(s[i] == '.'){
            is_dec = 1;
        }
        else if(s[i] >= '0' && s[i] <= '9'){
            if(is_dec){
                f = f + wei * (float)(s[i] - '0');
                wei = wei * 0.1f;
            }
            else{
                v = v * 10.0f + (float)(s[i] - '0');
            }
        }
    }
    v = v + f;
    if(is_neg) v = -v;
    return v;
}

// 把积累的文本当一条命令执行：start / stop / kp30 / ki0.5 / kd0 / 数字(目标速度)
void Process_Ascii_Line(void){
    if(ascii_idx == 0) return;
    ascii_buf[ascii_idx] = 0;                  // 补字符串结尾
    if(strcmp(ascii_buf, "start") == 0){
        motor_enable = 1;                      // 启动
    }
    else if(strcmp(ascii_buf, "stop") == 0){
        motor_enable = 0;                      // 停止
        target_spd = 0.0f;                     // 目标也归0
    }
    else if(ascii_buf[0]=='k' && ascii_buf[1]=='p' && ascii_buf[2]!=0){
        Kp = Parse_Number(&ascii_buf[2]);      // "kp30" 设置 Kp
        pid_i_l = pid_i_r = 0.0f;              // 换参数，积分清零重来
        pid_last_l = pid_last_r = 0.0f;
    }
    else if(ascii_buf[0]=='k' && ascii_buf[1]=='i' && ascii_buf[2]!=0){
        Ki = Parse_Number(&ascii_buf[2]);      // "ki0.5" 设置 Ki
        pid_i_l = pid_i_r = 0.0f;
    }
    else if(ascii_buf[0]=='k' && ascii_buf[1]=='d' && ascii_buf[2]!=0){
        Kd = Parse_Number(&ascii_buf[2]);      // "kd0" 设置 Kd
        pid_last_l = pid_last_r = 0.0f;
    }
    else if(ascii_buf[0]=='f' && ascii_buf[1]=='f' && ascii_buf[2]!=0){
        FF = Parse_Number(&ascii_buf[2]);      // "ff78" 设置前馈斜率
    }
    else if(ascii_buf[0]=='f' && ascii_buf[1]=='b' && ascii_buf[2]!=0){
        FF_bias = Parse_Number(&ascii_buf[2]); // "fb4458" 设置起步油
    }
    else if(ascii_buf[0] != 'k'){
        // 其它内容当数字，转成目标速度
        target_spd = Parse_Number(ascii_buf);
        if(target_spd >  1000.0f) target_spd =  1000.0f;   // 限幅到 ±1000
        if(target_spd < -1000.0f) target_spd = -1000.0f;
    }
    ascii_idx = 0;
    ascii_idle = 0;
}

// 逐字节解析文本命令（不用等回车也能执行）
void Parse_Ascii_Byte(uint8_t dat){
    if(dat == '\n' || dat == '\r' || dat == ' '){   // 回车/空格 = 一条命令结束
        Process_Ascii_Line();
    }
    else if(ascii_idx < 15){
        ascii_buf[ascii_idx++] = (char)dat;        // 存字符
        ascii_buf[ascii_idx] = 0;
        ascii_idle = 0;                            // 收到新字符，重置超时
        // start/stop 一凑齐就立刻执行，不用等回车
        if(strcmp(ascii_buf, "start") == 0){ motor_enable = 1; ascii_idx = 0; ascii_idle = 0; }
        else if(strcmp(ascii_buf, "stop") == 0){ motor_enable = 0; target_spd = 0.0f; ascii_idx = 0; ascii_idle = 0; }
    }
}

// 回传遥测：FireWater 文本协议（逗号分隔 + 换行结尾），VOFA+ 里选 FireWater 引擎
void Send_Telemetry(void){
    printf("%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d\n",
           target_spd, actual_spd, pwm_out, pid_i_l, lv, rv, Kp, Ki, Kd, (int)motor_enable);
}

// **************************** 代码区域 ****************************
void User_Main(void){

    // 初始化默认调试串口
    NEUQ_Debug_init();
    // 延迟初始化函数
    NEUQ_Delay_init();

    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6,Encoder_TIM3_PhaseB_PA7);
    NEUQ_Encoder_init(Encoder_TIM4,Encoder_TIM4_PhaseA_PB6,Encoder_TIM4_PhaseB_PB7);

    NEUQ_GPIO_init(PA5,GPIO_Mode_Output_PushPull,0);
    NEUQ_GPIO_init(PC7,GPIO_Mode_Output_PushPull,0);
    NEUQ_PWM_init(TIM2_CH3_PA2,1000,0);
    NEUQ_PWM_init(TIM8_CH1_PC6,1000,0);
    NEUQ_PIT_init_ms(PIT_CH1,10);

    // 使能串口接收中断（新增）
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 1);

    while(1){
    if(PIT1_flag == 1)
    {
      //读取编码器（灰排线反插后A/B反了，正转现在读正，不再取反；右轮暂未接）
      lv = NEUQ_Encoder_Read(Encoder_TIM3);
      rv = NEUQ_Encoder_Read(Encoder_TIM4);
      NEUQ_Encoder_Clear(Encoder_TIM3);
      NEUQ_Encoder_Clear(Encoder_TIM4);

      // 只接了一个轮子，实际速度直接用左轮
      actual_spd = (float)lv;

      if(motor_enable == 1)
      {
        float ff_bias = 0.0f;                          // 起步油：正转+/反转-，目标为0时不给
        if(target_spd > 0.0f)      ff_bias =  FF_bias;
        else if(target_spd < 0.0f) ff_bias = -FF_bias;

        //=====左轮PID控制 PA5方向 PA2 PWM=====
        float err_l = target_spd - lv;               // 1.误差
        float der_l = err_l - pid_last_l;            // 2.微分
        pid_last_l = err_l;

        // 3.先用上一次的积分算完整输出，看会不会顶满
        float pwm_l = FF*target_spd + ff_bias + Kp*err_l + Ki*pid_i_l + Kd*der_l;

        // 4.抗积分饱和：输出顶满、误差还在同方向顶就冻结积分；误差反向了就继续
        if(Ki > 0.001f){
            if(!((pwm_l >=  10000.0f && err_l > 0) || (pwm_l <= -10000.0f && err_l < 0))){
                pid_i_l += err_l;
                if(pid_i_l >  4000.0f) pid_i_l =  4000.0f;
                if(pid_i_l < -4000.0f) pid_i_l = -4000.0f;
            }
        }
        else{
            pid_i_l = 0.0f;                          // Ki=0 时积分没用，清零
        }

        // 5.再用新积分算最终 PWM，限幅
        pwm_l = FF*target_spd + ff_bias + Kp*err_l + Ki*pid_i_l + Kd*der_l;
        if(pwm_l >  10000.0f) pwm_l =  10000.0f;    // 上限：满油门正转
        if(pwm_l < -10000.0f) pwm_l = -10000.0f;    // 下限：满油门反转

        if(pwm_l >= 0) NEUQ_GPIO_write(PA5,1);      // 正转：PA5=1
        else           NEUQ_GPIO_write(PA5,0);      // 反转：PA5=0
        NEUQ_PWM_set_duty(TIM2_CH3_PA2,(uint32_t)fabs(pwm_l));   // 油门大小
        pwm_out = pwm_l;                            // 记录油门，回传看是否顶满(10000)

        //=====右轮PID控制 PC7方向 PC6 PWM=====（逻辑和左轮一样）
        float err_r = target_spd - rv;
        float der_r = err_r - pid_last_r;
        pid_last_r = err_r;

        // 先用上一次的积分算完整输出，看会不会顶满
        float pwm_r = FF*target_spd + ff_bias + Kp*err_r + Ki*pid_i_r + Kd*der_r;

        // 抗积分饱和：输出顶满、误差还在同方向顶就冻结积分；误差反向了就继续
        if(Ki > 0.001f){
            if(!((pwm_r >=  10000.0f && err_r > 0) || (pwm_r <= -10000.0f && err_r < 0))){
                pid_i_r += err_r;
                if(pid_i_r >  4000.0f) pid_i_r =  4000.0f;
                if(pid_i_r < -4000.0f) pid_i_r = -4000.0f;
            }
        }
        else{
            pid_i_r = 0.0f;
        }

        // 再用新积分算最终 PWM，限幅
        pwm_r = FF*target_spd + ff_bias + Kp*err_r + Ki*pid_i_r + Kd*der_r;
        if(pwm_r >  10000.0f) pwm_r =  10000.0f;
        if(pwm_r < -10000.0f) pwm_r = -10000.0f;

        if(pwm_r >= 0) NEUQ_GPIO_write(PC7,1);
        else           NEUQ_GPIO_write(PC7,0);
        NEUQ_PWM_set_duty(TIM8_CH1_PC6,(uint32_t)fabs(pwm_r));
      }
      else
      {
        // 停止：清积分、清误差，PWM=0（松油门，电机靠惯性滑行停下）
        pid_i_l = pid_last_l = 0.0f;
        pid_i_r = pid_last_r = 0.0f;
        pwm_out = 0.0f;                    // 油门也归0，波形不残留
        NEUQ_GPIO_write(PA5,0);
        NEUQ_GPIO_write(PC7,0);
        NEUQ_PWM_set_duty(TIM2_CH3_PA2,0);
        NEUQ_PWM_set_duty(TIM8_CH1_PC6,0);
      }

      PIT1_flag = 0;

      // 文本命令超时：超过100ms没收到新字符，就把积累的数字当一条命令
      if(ascii_idx > 0){
        ascii_idle++;
        if(ascii_idle >= 10){   // 10个PIT周期 = 100ms
          Process_Ascii_Line();
        }
      }

      // 每2个周期(20ms)回传一次遥测
      tx_count++;
      if(tx_count >= 2){
        tx_count = 0;
        Send_Telemetry();
      }
    }

    // 主循环里处理串口收到的字节（不打断上面的控制）
    while(box_index != box_head){
      uint8_t dat = receive_box[box_index];
      uint8_t was_idle = (rx_state == 0);   // 处理前是否不在二进制帧里
      Parse_Frame_Byte(dat);                // 二进制协议（原来的）
      if(was_idle && dat != FRAME_HEAD1){   // 不在帧里、也不是帧头，才当文本
        Parse_Ascii_Byte(dat);              // 文本命令（VOFA+ 滑块用）
      }
      box_index = box_index + 1;
      if(box_index >= BUF_MAX) box_index = 0;   // 读到头了回到开头
    }
  }
}
