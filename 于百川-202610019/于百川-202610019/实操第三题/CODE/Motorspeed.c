#include "NEUQ_lib_include.h"

static float integral = 0.0f;                   // 误差积分（仅中断里访问）
static float last_error = 0.0f;                 // 上一次误差（微分项用）
static float actual_lpf = 0.0f; 
static Speed_Control_t spe;        // 本文件私有的唯一实例

void Speed_Init(void){
    spe.running = 0;
    spe.target  = 0;
    spe.actual  = 0;
    spe.kp = 0.0f;
    spe.ki = 0.0f;
    spe.kd = 0.0f;

    integral = 0.0f;
    last_error = 0.0f;
    actual_lpf = 0.0f;

    NEUQ_Encoder_init(SPEED_ENC_DEV, SPEED_ENC_PHASE_A, SPEED_ENC_PHASE_B);
    NEUQ_Motor_Stop();
}

void Speed_Start(void){
    spe.running = 1;
    integral = 0.0f;
    last_error = 0.0f;    
    actual_lpf = 0.0f;
}

void Speed_Stop(void){
    spe.running = 0;
    integral = 0.0f;
    last_error = 0.0f; 
    actual_lpf = 0.0f;
    NEUQ_Motor_Stop();
}

void Speed_Target(int32_t target){
    spe.target = target;
}

//修改 PID 参数（仅停止状态允许，运行中直接忽略） 
void Speed_PID(float kp, float ki, float kd){
    if (spe.running) return;                   // 题目要求：PID 只能在停止状态下修改

    spe.kp = kp;
    spe.ki = ki;
    spe.kd = kd;

    integral   = 0.0f;
    last_error = 0.0f;
}

/*------------------ 10ms 周期任务：测速 -> PID -> 更新PWM ------------------*/
void SpeedControl_Task_10ms(void)
{
    int32_t cnt;
    float   error, diff, out;
    float   kp, ki, kd;
    int32_t target;

    /* 1. 编码器测速：读增量 -> 清零 -> 换算成"计数/秒" */
   /* cnt = NEUQ_Encoder_Read(SPEED_ENC_DEV);
    NEUQ_Encoder_Clear(SPEED_ENC_DEV);
    spe.actual = cnt * SPEED_SCALE;*/

    cnt = NEUQ_Encoder_Read(SPEED_ENC_DEV);
    NEUQ_Encoder_Clear(SPEED_ENC_DEV);

    {
        float raw = (float)(cnt * SPEED_SCALE);   // 本拍原始速度（整数脉冲带来的台阶噪声在这里）
        const float alpha = 0.3f;                 // 滤波系数 0~1：越大越跟手，越小越平滑
        actual_lpf = alpha * raw + (1.0f - alpha) * actual_lpf;  // 新拍×30% + 历史×70%
        spe.actual = (int32_t)actual_lpf;         // 滤波结果回填，PID 和 VOFA 回传都用它
    }


    /* 2. 停止状态：不计算PID，直接刹停 */
    if (!spe.running) {
        NEUQ_Motor_Stop();
        integral = 0.0f;
        last_error = 0.0f;
        return;
    }

    /* 3. 取参数（volatile 变量一次性拷到局部，避免中途被改） */
    target = spe.target;
    kp = spe.kp;
    ki = spe.ki;
    kd = spe.kd;

    /* 4. 位置式 PID */
    error = (float)(target - spe.actual);      // 误差 = 目标 - 实际

    integral += error;                          // 积分项累加
    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;   // 积分限幅
    if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

    diff = error - last_error;                  // 微分项 = 本次误差 - 上次误差
    last_error = error;

    out = kp * error + ki * integral + kd * diff;

    /* 5. 输出限幅并写 PWM */
    if (out >  PID_OUT_MAX) out =  PID_OUT_MAX;
    if (out < -PID_OUT_MAX) out = -PID_OUT_MAX;
    Motor_SetOutput((int32_t)out);
}

/*------------------ 只读接口（与 Motorspeed.h 声明一致） ------------------*/
uint8_t Speed_GetRunning(void) { return spe.running; }
int32_t Speed_GetTarget (void) { return spe.target;  }
int32_t Speed_GetActual (void) { return spe.actual;  }
float   Speed_GetKP     (void) { return spe.kp;      }
float   Speed_GetKI     (void) { return spe.ki;      }
float   Speed_GetKD     (void) { return spe.kd;      }
