#include "NEUQ_lib_include.h"
#include <math.h>

#define SIXAXIS_SAMPLE_PERIOD_MS 5      //采样周期
#define SIXAXIS_DISPLAY_INTERVAL 10     //每 10 个采样周期刷新一次数据显示
#define SIXAXIS_FLASH_SECTOR 510        //滤波参数保存扇区
#define SIXAXIS_ALPHA_MIN 0.01f         //滤波平滑系数下限
#define GRAVITY_MS2 9.80665f            //1g 对应的重力加速度（m/s^2），用于把加速度 g 值换算成 m/s^2
#define DEG_TO_RAD 0.01745329252f       //角度转弧度：PI/180，用于把陀螺仪角速度(度/秒)转成弧度/秒
#define RAD_TO_DEG 57.2957795131f       //弧度转角度：180/PI，用于把四元数解出的欧拉角转成度
#define MAHONY_KP  2.0f                 //Mahony 互补滤波比例增益
#define GYRO_Z_DEADZONE_DPS 0.3f        //航向角速度死区（度/秒）

//模块状态变量
static volatile uint32_t sample_count = 0; //采样计数
static uint32_t sample_done = 0;           //主循环已处理的采样数

static int16_t raw_ax,raw_ay,raw_az;     //加速度计原始值
static int16_t raw_gx,raw_gy,raw_gz;     //陀螺仪原始值
static float flt_ax,flt_ay,flt_az;       //加速度计滤波后
static float flt_gx,flt_gy,flt_gz;       //陀螺仪滤波后

static float roll, pitch, yaw;           //姿态角（单位：度）
//四元数姿态解算状态：四元数 q0(标量w) q1(x) q2(y) q3(z)，初值 q0=1 代表“零姿态”
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
//陀螺仪三轴零偏（度/秒），上电静置标定得到，用于消除静态漂移
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;      //原 Z 轴零偏，扩展为三轴

static int32_t saved_filter_coef;          //最近一次已写入 Flash 的滤波系数
static uint16_t display_tick;              //显示节流计数
static MenuID last_page;                   //上一次所在页面

//低通滤波

static float filter_alpha(void){
    float a = 1.0f - params.filter_coef / 100.0f;      //alpha = 1 - filter_coef/100，系数越大平滑越强（0=不滤波，100≈最强）
    if(a < SIXAXIS_ALPHA_MIN) a = SIXAXIS_ALPHA_MIN;
    if(a > 1.0f) a = 1.0f;
    return a;
}

//加速度计滤波系数
static float filter_alpha_accel(void){
    float a = 1.0f - (params.filter_coef * 0.5f) / 100.0f;   //系数减半 => 平滑更轻，范围约 [0.5, 1.0]
    if(a > 1.0f) a = 1.0f;                                   //filter_coef=0 时 a=1（不滤波）
    return a;
}

//Flash
static void flash_load(void){
    int32_t val = 0;
    if(NEUQ_Flash_read_sector(SIXAXIS_FLASH_SECTOR,(uint8_t*)&val, 4) == NEUQ_FLASH_OK){
        if(val >= 0 && val <= 100){        //校验滤波系数在0~100
            params.filter_coef = val;
        }
    }
    saved_filter_coef = params.filter_coef; //同步已保存值
}

//记录已保存值
static void flash_save(void){
    int32_t val = params.filter_coef;
    NEUQ_Flash_write_sector(SIXAXIS_FLASH_SECTOR,(uint8_t*)&val, 4);
    saved_filter_coef = val;               
}

//陀螺仪零偏标定
static void gyro_zero_calibrate(void){
    //三轴零偏累加器
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int n = 500;                         //采样次数
    for(int i = 0; i < n; i++){
        NEUQ_MPU6050_read_data();
        sum_x += mpu6050_gyro_x;         //累加 X 轴
        sum_y += mpu6050_gyro_y;         //累加 Y 轴
        sum_z += mpu6050_gyro_z;         //累加 Z 轴
        NEUQ_Delay_ms(1);                  //每次间隔约 1ms，共约 500ms 完成标定
    }
    gyro_x_offset = MPU6050_gyro_transition((int16_t)(sum_x / n));
    gyro_y_offset = MPU6050_gyro_transition((int16_t)(sum_y / n));
    gyro_z_offset = MPU6050_gyro_transition((int16_t)(sum_z / n));
}


//用初始加速度初始化四元数
static void quaternion_init(void){
    NEUQ_MPU6050_read_data();   //读一次最新数据
    float ax = (float)mpu6050_acc_x;
    float ay = (float)mpu6050_acc_y;
    float az = (float)mpu6050_acc_z;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1.0f) norm = 1.0f;   //防止除零（静止时重力约 1g，模长不应为 0）
    ax /= norm; ay /= norm; az /= norm;

    //由归一化加速度求初始 roll/pitch（与下方四元数->欧拉角的 ZYX 约定一致）
    float init_roll  = atan2f(ay, az);
    float init_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    //航向以当前方向为相对 0 参考，故下面直接用 yaw=0 代入 ZYX 四元数公式（cy=1,sy=0 已简化）

    //ZYX 欧拉角(roll, pitch, yaw=0) -> 四元数
    float cr = cosf(init_roll * 0.5f),  sr = sinf(init_roll * 0.5f);
    float cp = cosf(init_pitch * 0.5f), sp = sinf(init_pitch * 0.5f);
    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    //四元数归一化（保证模长为 1）
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(norm > 1e-6f){
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }
}

//Mahony 互补滤波姿态更新
static void mahony_update(void){
    float gx, gy, gz;   //去零偏后的角速度（弧度/秒）
    float ax, ay, az;   //归一化加速度（单位向量，其方向即重力方向）
    float norm, halfT;
    float halfvx, halfvy, halfvz;   //四元数估计出的重力方向（导航系[0,0,1]在机体系下的投影）
    float halfex, halfey, halfez;   //实测重力与估计重力之间的叉乘误差
    float qa, qb, qc;               //四元数积分前的临时缓存

    //1. 陀螺仪原始值 -> 度/秒 -> 减零偏 -> 弧度/秒（Mahony 内部用弧度/秒）
    float gx_dps = MPU6050_gyro_transition((int16_t)flt_gx) - gyro_x_offset;
    float gy_dps = MPU6050_gyro_transition((int16_t)flt_gy) - gyro_y_offset;
    float gz_dps = MPU6050_gyro_transition((int16_t)flt_gz) - gyro_z_offset;
    //🐕 航向死区：加速度计只修正横滚/俯仰、无法修正航向，静止时残余零偏会让航向持续漂移，
    //🐕 因此把小于阈值的航向角速度归零（阈值略大于标定残余即可，转动时远超该值不会被误伤）
    if(gz_dps < GYRO_Z_DEADZONE_DPS && gz_dps > -GYRO_Z_DEADZONE_DPS){
        gz_dps = 0.0f;
    }
    gx = gx_dps * DEG_TO_RAD;
    gy = gy_dps * DEG_TO_RAD;
    gz = gz_dps * DEG_TO_RAD;

    //2. 加速度归一化（静止时其方向即重力方向，用于修正姿态；单位会在归一化时抵消）
    norm = sqrtf(flt_ax*flt_ax + flt_ay*flt_ay + flt_az*flt_az);
    if(norm > 1e-6f){
        ax = flt_ax / norm; ay = flt_ay / norm; az = flt_az / norm;
    }else{
        ax = 0.0f; ay = 0.0f; az = 1.0f;   //加速度全零时退化为只靠陀螺仪积分
    }

    //3. 由当前四元数估计重力方向（导航系 [0,0,1] 旋转到机体系后的投影）
    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5f + q3*q3;

    //4. 实测重力 与 估计重力 做叉乘，得到姿态误差向量
    halfex = (ay*halfvz - az*halfvy);
    halfey = (az*halfvx - ax*halfvz);
    halfez = (ax*halfvy - ay*halfvx);

    //5. 比例反馈：用误差修正陀螺仪角速度，把姿态“拉回”到与重力一致（消除陀螺仪积分漂移）
    gx += MAHONY_KP * halfex;
    gy += MAHONY_KP * halfey;
    gz += MAHONY_KP * halfez;

    //6. 一阶积分更新四元数：dq/dt = 0.5 * q ⊗ [0, gx, gy, gz]
    halfT = 0.5f * (SIXAXIS_SAMPLE_PERIOD_MS / 1000.0f);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz) * halfT;
    q1 += ( qa*gx + qc*gz - q3*gy) * halfT;
    q2 += ( qa*gy - qb*gz + q3*gx) * halfT;
    q3 += ( qa*gz + qb*gy - qc*gx) * halfT;

    //7. 四元数归一化（消除数值积分带来的模长误差）
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(norm > 1e-6f){
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }

    //8. 四元数 -> 欧拉角（ZYX 约定），单位：度
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    pitch = asinf(2.0f*(q0*q2 - q3*q1)) * RAD_TO_DEG;
    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;

    //9. 航向角规整到 [-180, 180]（方便显示与判断）
    while(yaw > 180.0f)  yaw -= 360.0f;
    while(yaw < -180.0f) yaw += 360.0f;
}

//采样后滤波再姿态解算
static void sample_process(void){
    //读取六轴原始数据
    NEUQ_MPU6050_read_data();
    raw_ax = mpu6050_acc_x;
    raw_ay = mpu6050_acc_y;
    raw_az = mpu6050_acc_z;
    raw_gx = mpu6050_gyro_x;
    raw_gy = mpu6050_gyro_y;
    raw_gz = mpu6050_gyro_z;

    //六轴一阶低通滤波（加速度计/陀螺仪分开滤波：陀螺用原系数更平滑，加速度计更轻以减小姿态滞后）
    float a_acc = filter_alpha_accel();
    float a_gyr = filter_alpha();
    flt_ax += a_acc*((float)raw_ax - flt_ax);
    flt_ay += a_acc*((float)raw_ay - flt_ay);
    flt_az += a_acc*((float)raw_az - flt_az);
    flt_gx += a_gyr*((float)raw_gx - flt_gx);
    flt_gy += a_gyr*((float)raw_gy - flt_gy);
    flt_gz += a_gyr*((float)raw_gz - flt_gz);

    //旧姿态解算
    //const double RAD_TO_DEG = 57.29577951308232; //把弧度转化成角度
    //double ax = (double)flt_ax;
    //double ay = (double)flt_ay;
    //double az = (double)flt_az;
    //roll  = (float)(atan2(ay, az) * RAD_TO_DEG);  //横滚角
    //pitch = (float)(atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG);  //俯仰角
 
    //float wz = MPU6050_gyro_transition((int16_t)flt_gz) - gyro_z_offset;
    //yaw += wz * (SIXAXIS_SAMPLE_PERIOD_MS / 1000.0f);  //偏航角

    //边界限制
    //while(yaw > 180.0f)  yaw -= 360.0f;
    //while(yaw < -180.0f) yaw += 360.0f;

    mahony_update();
}

//VOFA
static void vofa_send(void){
    int16_t raw_val;
    float   flt_val;
    switch(params.send_axis){
        case 1: 
            raw_val = raw_ax;flt_val = flt_ax; 
            break;   //Ax
        case 2: 
            raw_val = raw_ay;flt_val = flt_ay; 
            break;   //Ay
        case 3: 
            raw_val = raw_az;flt_val = flt_az; 
            break;   //Az
        case 4: 
            raw_val = raw_gx;flt_val = flt_gx; 
            break;   //Gx
        case 5: 
            raw_val = raw_gy;flt_val = flt_gy; 
            break;   //Gy
        case 6: 
            raw_val = raw_gz;flt_val = flt_gz; 
            break;   //Gz
        default: 
            return;                 
    }

    uint8_t buf[12];
    float f = (float)raw_val;           
    memcpy(&buf[0], &f, 4);              //ch0：原始值
    memcpy(&buf[4], &flt_val, 4);        //ch1：滤波后
    buf[8]  = 0x00;                      //帧尾（?）
    buf[9]  = 0x00;
    buf[10] = 0x80;
    buf[11] = 0x7F;
    NEUQ_Uart_write_data(UART_1, buf, 12); 
}

//数据显示
static void fmt_1dec(float v, char *out){
    int32_t tenths = (int32_t)(v * 10.0f);   //把值放大 10 倍取整，得到十分位表示
    char sign = ' ';
    if(tenths < 0){ sign = '-'; tenths = -tenths; }
    int32_t ip = tenths / 10;                //整数部分
    int32_t fp = tenths % 10;                //十分位
    snprintf(out, 16,"%c%3ld.%ld",sign,(long)ip,(long)fp);
}

//拼出显示结果
static void fmt_fixed(float v, int32_t decimals, int32_t width, char *out){
    int32_t scale = 1;
    for(int32_t i = 0; i < decimals; i++) scale *= 10;

    float scaled = v * (float)scale;
    int32_t val = (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f)); 

    char sign = ' ';
    if(val < 0){sign = '-';val = -val; }

    int32_t ip = val / scale;                 //整数部分
    int32_t fp = val % scale;                 //小数部分

    //拼出符号 + 整数 + 小数点 + 小数
    char tmp[24];
    int32_t len = 0;
    tmp[len++] = sign;

    char digits[12];
    int32_t n = 0;
    do {
        digits[n++] = (char)('0' + ip % 10); ip /= 10; 
    }while(ip > 0);
    while(n > 0){
        tmp[len++] = digits[--n];
    }
    if(decimals > 0){
        tmp[len++] = '.';
        char frac[12];
        for(int32_t i = 0; i < decimals; i++){ frac[i] = (char)('0' + fp % 10); fp /= 10; }
        for(int32_t i = decimals - 1; i >= 0; i--) tmp[len++] = frac[i];
    }

    int32_t pad = width - len;
    if(pad > 0){
        for(int32_t i = 0; i < pad; i++){
            out[i] = ' ';
        }
        for(int32_t i = 0; i < len; i++){
            out[pad + i] = tmp[i];
            out[width] = '\0';
        }
    }else{
        for(int32_t i = 0; i < len; i++){
            out[i] = tmp[i];
            out[len] = '\0';
        }
    }
}

//六轴物理量显示
static void draw_phys_row(uint16_t y, const char *name, float value, int32_t decimals, int32_t width, const char *unit){
    char num[24];
    char line[40];
    fmt_fixed(value, decimals, width, num);
    snprintf(line, sizeof(line), "%s: %s %s", name, num, unit);
    NEUQ_LCD200_DisplayString(8, y, line);
}

//姿态角显示
static void draw_angle_row(uint16_t y, const char *name, float deg){
    char num[16];
    char line[32];
    fmt_1dec(deg, num);
    snprintf(line, sizeof(line), "%s: %s deg", name, num);
    NEUQ_LCD200_DisplayString(8, y, line);
}

//六轴数据显示页
static void draw_sixaxis_page(uint8_t full_redraw){
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    if(full_redraw){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(8,8,"6-Axis Data");
    }
    //加速度
    draw_phys_row(48,"Ax", MPU6050_acc_transition(raw_ax) * GRAVITY_MS2,2,6,"m/s^2");
    draw_phys_row(80,"Ay", MPU6050_acc_transition(raw_ay) * GRAVITY_MS2,2,6,"m/s^2");
    draw_phys_row(112,"Az", MPU6050_acc_transition(raw_az) * GRAVITY_MS2,2,6,"m/s^2");
    //角速度
    draw_phys_row(144,"Gx", MPU6050_gyro_transition(raw_gx),1,7,"deg/s");
    draw_phys_row(176,"Gy", MPU6050_gyro_transition(raw_gy),1,7,"deg/s");
    draw_phys_row(208,"Gz", MPU6050_gyro_transition(raw_gz),1,7,"deg/s");
}

//姿态解算显示页
static void draw_attitude_page(uint8_t full_redraw){
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    if(full_redraw){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(8,8,"Attitude");
    }
    draw_angle_row(48,"Roll",roll);
    draw_angle_row(80,"Pitch",pitch);
    draw_angle_row(112,"Yaw",yaw);
}

//对外接口
void NEUQ_SixAxis_Init(void){
    NEUQ_Flash_init();                      
    flash_load();                               //从 Flash 加载上次保存的滤波系数

    NEUQ_MPU6050_init();                     
    NEUQ_Delay_ms(100);

    NEUQ_Uart_init(UART_1, UART_BAUD_1000000);  //VOFA+ 上传串口（UART_1: PA2/PA3，1M 波特率）

    gyro_zero_calibrate();                      //陀螺仪三轴零偏标定
    quaternion_init();                          //用初始加速度初始化四元数

    display_tick = 0;
    last_page = MENU_NONE;               
}

//采样计数自增
void NEUQ_SixAxis_TimeCallback(void){
    sample_count++;
}

void NEUQ_SixAxis_Task(void){
    //采样
    while(sample_done < sample_count){
        sample_done++;
        sample_process();                        //读传感器六轴滤波再姿态解算

    //波形上传
        if(params.send_enable && params.send_axis >= 1 && params.send_axis <= 6){
            vofa_send();
        }

    //数据页动态刷新
        display_tick++;
        MenuID page = NEUQ_Menu_GetPage();
        if(page == MENU_DISPLAY || page == MENU_ATTITUDE){
            if(page != last_page || display_tick >= SIXAXIS_DISPLAY_INTERVAL){
                display_tick = 0;
                uint8_t full = (page != last_page);  //切页 = 首次进入，需清屏
                if(page == MENU_DISPLAY){
                    draw_sixaxis_page(full);
                }else{
                    draw_attitude_page(full);
                }
            }
        }
        last_page = page;
    }

    //滤波系数持久化，
    if(params.filter_coef != saved_filter_coef){
        flash_save();
    }
}
