#include "NEUQ_lib_include.h"
#include "NEUQ_data.h"

/*
  代码分工（通信/调参不得暂停电机控制）：
    [定周期中断] NEUQ_Control()        —— 编码器测速 -> PID -> 更新 PWM，10ms 一次；
                                          由 User/NEUQ_Interrupts.c 的
                                          NEUQ_PIT_1_CALLBACK() 调用（TIM6，优先级 0）
    [串口中断]   NEUQ_Uart_Rx_Handler() —— 只把字节搬进环形缓冲，立刻返回，绝不解析
    [主循环]     NEUQ_Comm_Update()     —— 逐字节解包（支持一帧分次到达/连续多帧）+ 回传

  闭环、限幅、测速全部在中断里完成，主循环只做通信；因此连续 30s 收发、
  在线调速、在线改 PID 都不会让控制曲线中断。
*/

/*控制节拍与限幅 */
#define CTRL_PERIOD_MS      10u          /* 控制周期 10ms -> 100Hz 控制与回传 */
#define PWM_DUTY_MAX        10000u       /* PWM 满量程 */
#define TARGET_SPEED_LIMIT  100.0f       /* 目标转速限幅 rad/s，防止误发大值超限 */
#define GAIN_LIMIT          100000.0f    /* PID 参数上限 */
#define PID_DELTA_LIMIT     6000.0f      /* 单周期 PWM 增量限幅，抑制突跳 */


// 电机 A: PWM = PC7 (TIM8_CH2)   DIR = PA5
//电机 B: PWM = PC6 (TIM8_CH1)   DIR = PA2
  
#define MOTOR_HAS_DIR_PINS  1                   
#define MOTOR_A_DIR         PA5                  /* 电机 A 方向脚 */
#define MOTOR_B_DIR         PA2                  /* 电机 B 方向脚 */
#define MOTOR_DIR_FORWARD   GPIO_Low             /* 正转方向脚电平；转向相反就改成 GPIO_High */
#define MOTOR_DIR_REVERSE   GPIO_High            /* 反转方向脚电平 */

#define MOTOR_A_PWM         TIM8_CH2_PC7         
#define MOTOR_B_PWM         TIM8_CH1_PC6         

#define ENC_B_SIGN          (+1.0f)              /* 两电机对装（转向相反）时改成 -1.0f，否则相加会抵消 */

/* 机械参数与测速单位换算 */
#define ENCODER_PPR         11.0f        /* 编码器线数（每圈脉冲数） */
#define ENCODER_MULT        4.0f         /* 定时器编码器模式 4 倍频 */
#define GEAR_RATIO          10.0f        /* 减速比：电机轴 30 圈 = 输出轴 1 圈 */
#define TWO_PI              6.2831853f
#define COUNTS_PER_REV      (ENCODER_PPR * ENCODER_MULT * GEAR_RATIO)   /* 输出轴一圈计数 = 1560 */
/* 每 10ms 的计数 -> rad/s：先换算成"每秒计数"，再按 每圈计数 = 2π rad 折算 */
#define SPEED_SCALE         ((1000.0f / (float)CTRL_PERIOD_MS) * (TWO_PI / COUNTS_PER_REV))


/* PID 参数与测速处理 */
#define DEFAULT_KP          100.0f       /* 上电初始值，可在停止状态下用字节包覆盖 */
#define DEFAULT_KI          1.0f
#define DEFAULT_KD          0.0f

/* 测速用"滚动窗口平均"代替一阶低通：
     分辨率 = 0.4028 / N  rad/s（N=1 时就是原来的量化台阶 0.40 rad/s），
     N=5 -> 分辨率约 0.08 rad/s，滞后约 20ms（想要响应更快可改 3） */
#define SPEED_WINDOW_N      5u
#define SPEED_LIMIT         2000.0f       /* 实际转速限幅 rad/s（防异常值污染 PID 和曲线） */

/*开环测试*/
/* 置 1 后忽略所有上位机命令，固定输出 MOTOR_TEST_DUTY，用来单独验证电机接线与转向 */
#define MOTOR_OPENLOOP_TEST 0
#define MOTOR_TEST_DUTY     5000u

/* 死区补偿*/
/* 电机必须跨过“启动死区”才会转（摩擦 + 驱动阈值的固有特性）。
   只用 PID 的话：Kp 太小或积分还没爬上来时，输出会一直卡在死区以下 → 电机不转。
   做法：PID 输出非零时直接加一个偏置，让占空比一步跨过死区，PID 只负责修正。

  标定方法：把 MOTOR_OPENLOOP_TEST 置 1，改 MOTOR_TEST_DUTY 从 1000 往上试，
     找出“刚好开始转”的占空比，填进 PWM_DEADZONE（保守可再加 10%）。
     把 PWM_DEADZONE 置 0 就是关闭补偿（回到纯 PID）。 */
#define PWM_DEADZONE        2500u     /* 电机起转的最小 PWM（占空比，按实测改） */
#define PWM_DZ_THRESHOLD    100.0f    /* PID 输出低于此值视为“不要转”，直接给 0（防零速爬行） */

/*========================== 全局状态 ==========================*/
volatile float   target_speed  = 0.0f;   /* 当前生效目标转速 rad/s（就是 PID 的给定） */
volatile float   current_speed = 0.0f;   /* 实际转速 rad/s（编码器测得） */
volatile float   pwm_output    = 0.0f;   /* 当前 PWM 输出 -10000~10000（负值=反转） */
volatile uint8_t motor_running = 0;      /* 0=停机（上电默认，等待启动命令） 1=运行 */
IncrementalPID   speed_pid;              /* 速度环 PID */

/*========================== 内部变量 ==========================*/
static volatile uint8_t  sample_flag   = 0;    /* 中断置 1：本周期已更新，主循环去回传 */
static volatile uint8_t  spd_reset_req = 0;    /* 主循环请求清测速窗口（由中断执行） */

/* 测速滚动窗口（只在 10ms 控制中断里读写） */
static float             spd_win[SPEED_WINDOW_N];   /* 最近 N 个周期的编码器计数 */
static float             spd_sum = 0.0f;            /* 窗口内计数之和 */
static uint8_t           spd_idx = 0u;              /* 下一个要被覆盖的位置 */

/*---------------- 串口接收环形缓冲（串口中断写、主循环读） ----------------*/
#define RX_FIFO_SIZE  128u
static volatile uint8_t  rx_buf[RX_FIFO_SIZE];
static volatile uint16_t rx_head = 0;          /* 写指针（串口中断） */
static volatile uint16_t rx_tail = 0;          /* 读指针（主循环） */

/*---------------- 组帧状态（只在主循环里使用） ----------------*/
static uint8_t  rx_frame[PACK_LEN];            /* 正在组装的帧 */
static uint8_t  rx_index = 0u;                 /* 已收字节数 */

/*========================== 内部函数声明 ==========================*/
static void motor_output(float out);
static void reset_control_state(void);
static void frame_feed(uint8_t byte);
static void frame_apply(const uint8_t *frame);
static void telemetry_send(void);

/*开机初始化：把电机、编码器、控制定时器、串口接收全部打开
  上电后电机保持停机、目标为 0，等待上位机下发启动命令*/
void NEUQ_data_init(void){

    /* 1) 方向脚：PA5 / PA2， GPIO 推挽输出（方向电平见 MOTOR_DIR_FORWARD） */
#if MOTOR_HAS_DIR_PINS
    NEUQ_GPIO_init(MOTOR_A_DIR, GPIO_Mode_Output_PushPull, MOTOR_DIR_FORWARD);
    NEUQ_GPIO_init(MOTOR_B_DIR, GPIO_Mode_Output_PushPull, MOTOR_DIR_FORWARD);
#endif
    
    /* 2) 电机 PWM：1kHz，初始占空比 0 */
    NEUQ_PWM_init(MOTOR_A_PWM, 1000, 0);
    NEUQ_PWM_init(MOTOR_B_PWM, 1000, 0);

    /* 3) 编码器*/
    NEUQ_Encoder_init(Encoder_TIM3, Encoder_TIM3_PhaseA_PA6, Encoder_TIM3_PhaseB_PA7);
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);

    /* 4) 速度PID 初值*/
    Ipid_parameters_setting(&speed_pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    speed_pid.max_delta_pwm = PID_DELTA_LIMIT;

    /* 5) 10ms 定周期中断，作为控制节拍（优先级 0，保证不被通信打断） */
    NEUQ_PIT_init_ms(PIT_CH1, CTRL_PERIOD_MS);

    /* 6) 打开串口接收中断：只把字节搬进缓冲，解析交给主循环 */
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);//串口使能函数，串口优先级5是最低

    /* 7) 上电状态：停机 / 目标 0 / 输出 0 */
    motor_running = 0;
    target_speed  = 0.0f;
    reset_control_state();
    motor_output(0.0f);
}

/*10ms 定周期中断里调用 编码器测速 -> PID -> 更新 PWM
 printf / 串口等待等耗时操作*/
void NEUQ_Control(void){

    /* 第 1 步：编码器测速（滚动窗口平均）*/
    if(spd_reset_req){                                       /* 启停时主循环请求复位窗口 */
        spd_reset_req = 0u;
        for(uint8_t i = 0u; i < SPEED_WINDOW_N; i++) spd_win[i] = 0.0f;
        spd_sum = 0.0f;
        spd_idx = 0u;
    }

    float cnt = (float)NEUQ_Encoder_Read(Encoder_TIM3);      /* 本周期的编码器计数 */
    NEUQ_Encoder_Clear(Encoder_TIM3);
    cnt += ENC_B_SIGN * (float)NEUQ_Encoder_Read(Encoder_TIM4);
    NEUQ_Encoder_Clear(Encoder_TIM4);

    spd_sum -= spd_win[spd_idx];                             /* 滑出最旧的一个周期 */
    spd_win[spd_idx] = cnt;
    spd_sum += cnt;                                          /* 滑入本周期 */
    spd_idx++;
    if(spd_idx >= SPEED_WINDOW_N) spd_idx = 0u;

    /* 窗口内平均计数 × SPEED_SCALE = rad/s；分辨率 0.4028/N，低速不再被量化吃掉 */
    float v = (spd_sum / (float)SPEED_WINDOW_N) * SPEED_SCALE;
    if(v >  SPEED_LIMIT) v =  SPEED_LIMIT;                   /* 实际转速限幅 */
    if(v < -SPEED_LIMIT) v = -SPEED_LIMIT;
    current_speed = v;

    /*第 2 步：PID 计算并限幅*/
#if MOTOR_OPENLOOP_TEST
    pwm_output = (float)MOTOR_TEST_DUTY;                  /* 自检：固定占空比，忽略闭环 */
#else
    if(motor_running){
        /* 增量式 PID：输出累加在 pwm_output 上，消除稳态误差 */
        pwm_output += IncrementalPID_Update(&speed_pid, target_speed, current_speed);
        if(pwm_output >  (float)PWM_DUTY_MAX) pwm_output =  (float)PWM_DUTY_MAX;
        if(pwm_output < -(float)PWM_DUTY_MAX) pwm_output = -(float)PWM_DUTY_MAX;
    }else{
        pwm_output = 0.0f;                                /* 停机：输出 0，电机不转 */
    }
#endif

    /*更新 PWM 输出*/
    motor_output(pwm_output);

    /*通知主循环回传本周期数据*/
    sample_flag = 1;
}

/*串口中断里调用  只把字节塞进环形缓冲，不做任何解析，保证不阻塞*/
void NEUQ_Uart_Rx_Handler(uint8_t dat){
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_FIFO_SIZE);
    if(next != rx_tail){        /* 缓冲没满才存；满了直接丢弃，绝不阻塞中断 */
        rx_buf[rx_head] = dat;
        rx_head = next;
    }
}

/*主循环里一直调用  解包上位机字节包 + 回传曲线数据
  放在主循环，通信再频繁也不会影响中断里的闭环*/
void NEUQ_Comm_Update(void){

    // 1) 把缓冲里的字节全部喂给组帧状态机（支持分次到达的一帧、连续多帧） //可以试试用FIFO替代
    while(rx_tail != rx_head){
        uint8_t dat = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_FIFO_SIZE);
        frame_feed(dat);
    }

    // 2) 有新的控制周期数据就回传一帧（回传在中断之外进行，不影响控制） 
    if(sample_flag){
        sample_flag = 0;
        telemetry_send();
    }
}

/*
  把 PID 输出变成真正的电机动作：占空比 = |out| + 死区补偿，再限幅到 0~10000
    MOTOR_HAS_DIR_PINS=1 时：out<0 用方向脚切反转
    MOTOR_HAS_DIR_PINS=0 时：只有单向调速，负目标按 0 处理
*/
static void motor_output(float out){
    uint32_t duty;

#if MOTOR_HAS_DIR_PINS
    GPIO_Level dir = MOTOR_DIR_FORWARD;

    if(out < 0.0f){                             /* 负 = 反转 */
        dir = MOTOR_DIR_REVERSE;
        out = -out;
    }
    NEUQ_GPIO_write(MOTOR_A_DIR, dir);
    NEUQ_GPIO_write(MOTOR_B_DIR, dir);
#else
    /* 单向调速，负目标按 0 处理*/
    if(out < 0.0f) out = 0.0f;
#endif

#if !MOTOR_OPENLOOP_TEST
    /* 死区补偿：控制器不想要动就干净给 0；要动就直接加偏置跨过死区。
       （自检模式不补，这样 MOTOR_TEST_DUTY 就是真实占空比，方便标定死区） */
    if(out < PWM_DZ_THRESHOLD) out = 0.0f;
    else                       out += (float)PWM_DEADZONE;
#endif

    if(out > (float)PWM_DUTY_MAX) out = (float)PWM_DUTY_MAX;
    duty = (uint32_t)out;                       /* 0~10000 对应 0%~100% 占空比 */

    NEUQ_PWM_set_duty(MOTOR_A_PWM, duty);       /* 与 NEUQ_data_init() 初始化的通道保持一致 */
    NEUQ_PWM_set_duty(MOTOR_B_PWM, duty);
}

/*启停时清控制状态：编码器计数、滤波状态、输出与 PID 历史误差
  这样"启动"总是从零误差、零输出开始，不会带着上一次的积分突跳
  注意：调用顺序上要先清状态，最后再置 motor_running，避免中断用到半成品状态*/
static void reset_control_state(void){
    NEUQ_Encoder_Clear(Encoder_TIM3);
    NEUQ_Encoder_Clear(Encoder_TIM4);
    spd_reset_req = 1u;            /* 测速窗口交给控制中断清零（避免主循环与中断抢数组） */
    current_speed = 0.0f;
    pwm_output    = 0.0f;
    /* 清 PID 历史误差（Ipid_parameters_setting 会保留 Kp/Ki/Kd 并把误差清零） */
    Ipid_parameters_setting(&speed_pid, speed_pid.Kp, speed_pid.Ki, speed_pid.Kd);
    speed_pid.max_delta_pwm = PID_DELTA_LIMIT;
}


/*
  逐字节组帧状态机
     一帧收满 8 字节才处理
     连续多帧：处理完一帧后归零，紧接着组下一帧
     帧头错位：自动重新寻找 AA 55
    
*/
static void frame_feed(uint8_t byte){

    if(rx_index == 0u){                       /* 等待帧头 0xAA */
        if(byte == PACK_HEAD0) rx_frame[rx_index++] = byte;
        return;
    }
    if(rx_index == 1u){                       /* 等待帧头 0x55 */
        if(byte == PACK_HEAD1){
            rx_frame[rx_index++] = byte;
        }else{                                /* 帧头没对上，重新同步 */
            rx_index = 0u;
            if(byte == PACK_HEAD0) rx_frame[rx_index++] = byte;
        }
        return;
    }

    rx_frame[rx_index++] = byte;              /* 收 cmd / id / value */
    if(rx_index < PACK_LEN) return;           /* 一帧还没收全（8 字节） */
    rx_index = 0u;                            /* 已收全一帧 */
    NEUQ_GPIO_toggle(PA8);            /* 收到一帧就闪一下板载 LED，方便调试 */

    frame_apply(rx_frame);                    /* 收满一帧就直接生效 */
}

/*处理一条收满的完整帧（在主循环里执行，作用于中断共享变量）
    cmd = 0x01 启动 / 0x02 停止 / 0x03 设置参数/ 0x04 目标增量*/
static void frame_apply(const uint8_t *frame){
    uint8_t cmd = frame[2];
    uint8_t id  = frame[3];
    float   val;

    memcpy(&val, &frame[4], sizeof(float));   /* 小端 float，与上位机一致 */
    if(val != val) return;                    /* NaN 直接丢弃，别污染闭环 */

    switch(cmd){
        case CMD_START:
            if(!motor_running){
                /* 先清状态再置运行标志（中断只会在标志置 1 之后使用这些状态） */
                reset_control_state();
                motor_running = 1u;
            }
            break;

        case CMD_STOP:
            if(motor_running){
                motor_running = 0u;           /* 先停闭环 */
                reset_control_state();        /* 再清 PID/测速状态 */
                motor_output(0.0f);           /* 立即输出 0，不等下一个控制周期 */
            }
            break;

        case CMD_SET:  //这个是字节包中设置参数的部分
            if(id == ID_TARGET_SPEED){
                /* 目标转速：直接就是内部使用的 rad/s，做限幅；
                   运行中、停止中都可以改 */
                if(val >  TARGET_SPEED_LIMIT) val =  TARGET_SPEED_LIMIT;
                if(val < -TARGET_SPEED_LIMIT) val = -TARGET_SPEED_LIMIT;
                target_speed = val;
            }
            else if(id <= ID_KD){
                /* PID 参数：回传的就是当前真正生效的值，保证回传值 == 实际使用值 */
                if(!motor_running){
                    if(val < 0.0f || val > GAIN_LIMIT) return;
                    switch(id){
                        case ID_KP: speed_pid.Kp = val; break;
                        case ID_KI: speed_pid.Ki = val; break;
                        case ID_KD: speed_pid.Kd = val; break;
                        default: return;
                    }
                    /* 参数变了就清历史误差，避免旧积分被新参数放大后冲击输出 */
                    Ipid_parameters_setting(&speed_pid, speed_pid.Kp, speed_pid.Ki, speed_pid.Kd);
                    speed_pid.max_delta_pwm = PID_DELTA_LIMIT;
                }
            }
            break;

        case CMD_SET_STEP:
            /* 目标转速增量：target += value（可正可负），结果限幅
               —— 给 VOFA+ 快捷发送用：±1 / ±5 / ±0.5 点几下就到任意目标值 */
            val += target_speed;
            if(val >  TARGET_SPEED_LIMIT) val =  TARGET_SPEED_LIMIT;
            if(val < -TARGET_SPEED_LIMIT) val = -TARGET_SPEED_LIMIT;
            target_speed = val;
            break;

        default:
            break;                            /* 未知命令：忽略 */
    }
}

/*回传一帧 JustFloat：5 个 float + 4 字节帧尾 00 00 80 7F
  依次是 目标转速 / 实际转速 / Kp / Ki / Kd —— 全部是闭环里真正使用的变量*/
static void telemetry_send(void){
    static const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};   /* +inf，JustFloat 帧尾 */
    float tx[5];

    tx[0] = target_speed;        /* 通道 0：当前生效的目标转速（rad/s） */
    tx[1] = current_speed;       /* 通道 1：实际转速（rad/s） */
    tx[2] = speed_pid.Kp;        /* 通道 2：Kp */
    tx[3] = speed_pid.Ki;        /* 通道 3：Ki */
    tx[4] = speed_pid.Kd;        /* 通道 4：Kd */

    NEUQ_Uart_write_data(UART_0, (uint8_t *)tx, sizeof(tx));
    NEUQ_Uart_write_data(UART_0, (uint8_t *)tail, sizeof(tail));
}
