#include "NEUQ_lib_include.h"
#include <math.h>

#define SIXAXIS_SAMPLE_PERIOD_MS    5      //采样周期 5ms（200Hz）
#define SIXAXIS_DISPLAY_INTERVAL    10     //每 10 个采样周期刷新一次数据显示（约 50ms）
#define SIXAXIS_FLASH_SECTOR        510    //滤波参数保存扇区
#define SIXAXIS_ALPHA_MIN           0.01f  //滤波平滑系数下限：避免滤波过强导致输出完全不动

//模块状态变量
static volatile uint32_t sample_count = 0; //采样计数
static uint32_t sample_done = 0;           //主循环已处理的采样数

static int16_t raw_ax, raw_ay, raw_az;     //加速度计三轴原始值
static int16_t raw_gx, raw_gy, raw_gz;     //陀螺仪三轴原始值
static float   flt_ax, flt_ay, flt_az;     //加速度计三轴滤波后
static float   flt_gx, flt_gy, flt_gz;     //陀螺仪三轴滤波后

static float   roll, pitch, yaw;           //姿态角（单位：度）
static float   gyro_z_offset;              //陀螺仪 Z 轴零偏

static int32_t saved_filter_coef;          //最近一次已写入 Flash 的滤波系数
static uint16_t display_tick;              //显示节流计数（每 SIXAXIS_DISPLAY_INTERVAL 次采样刷新一次屏幕）
static MenuID   last_page;                 //上一次所在页面（检测页面切换，切换后立即刷一次屏）

//低通滤波
static float filter_alpha(void){
    float a = 1.0f - params.filter_coef / 100.0f;
    if(a < SIXAXIS_ALPHA_MIN) a = SIXAXIS_ALPHA_MIN;
    if(a > 1.0f) a = 1.0f;
    return a;
}

//Flash 参数持久化
static void flash_load(void){
    int32_t val = 0;
    if(NEUQ_Flash_read_sector(SIXAXIS_FLASH_SECTOR, (uint8_t*)&val, 4) == NEUQ_FLASH_OK){
        if(val >= 0 && val <= 100){        //合法性校验：滤波系数有效范围 0~100
            params.filter_coef = val;
        }
    }
    saved_filter_coef = params.filter_coef; //同步“已保存值”，避免开机就误触发一次写 Flash
}

static void flash_save(void){
    int32_t val = params.filter_coef;
    NEUQ_Flash_write_sector(SIXAXIS_FLASH_SECTOR, (uint8_t*)&val, 4);
    saved_filter_coef = val;               //记录已保存值
}

//陀螺仪零偏标定
static void gyro_zero_calibrate(void){
    int32_t sum = 0;
    int n = 200;
    for(int i = 0; i < n; i++){
        NEUQ_MPU6050_read_data();
        sum += mpu6050_gyro_z;
        NEUQ_Delay_ms(1);                  //每次间隔约 1ms，共约 200ms 完成标定
    }
    //把平均原始计数转换成 度/秒，得到零偏角速度
    gyro_z_offset = MPU6050_gyro_transition((int16_t)(sum / n));
}

//采样 + 滤波 + 姿态解算
static void sample_process(void){
    //读取六轴原始数据
    NEUQ_MPU6050_read_data();
    raw_ax = mpu6050_acc_x;
    raw_ay = mpu6050_acc_y;
    raw_az = mpu6050_acc_z;
    raw_gx = mpu6050_gyro_x;
    raw_gy = mpu6050_gyro_y;
    raw_gz = mpu6050_gyro_z;

    //六轴一阶低通滤波：filtered += alpha*(raw - filtered)
    float a = filter_alpha();
    flt_ax += a * ((float)raw_ax - flt_ax);
    flt_ay += a * ((float)raw_ay - flt_ay);
    flt_az += a * ((float)raw_az - flt_az);
    flt_gx += a * ((float)raw_gx - flt_gx);
    flt_gy += a * ((float)raw_gy - flt_gy);
    flt_gz += a * ((float)raw_gz - flt_gz);

    //姿态解算
    const double RAD_TO_DEG = 57.29577951308232;   //弧度转角度系数
    double ax = (double)flt_ax;
    double ay = (double)flt_ay;
    double az = (double)flt_az;
    roll  = (float)(atan2(ay, az) * RAD_TO_DEG);
    pitch = (float)(atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG);

    float wz = MPU6050_gyro_transition((int16_t)flt_gz) - gyro_z_offset;  //滤波后角速度扣零偏（度/秒）
    yaw += wz * (SIXAXIS_SAMPLE_PERIOD_MS / 1000.0f);
    //    偏航角环绕到 [-180, 180] 区间，避免长时间积分后数值无限增大
    while(yaw > 180.0f)  yaw -= 360.0f;
    while(yaw < -180.0f) yaw += 360.0f;
}

//VOFA+ 波形上传
static void vofa_send(void){
    int16_t raw_val;
    float   flt_val;

    switch(params.send_axis){
        case 1: raw_val = raw_ax; flt_val = flt_ax; break;   //Ax
        case 2: raw_val = raw_ay; flt_val = flt_ay; break;   //Ay
        case 3: raw_val = raw_az; flt_val = flt_az; break;   //Az
        case 4: raw_val = raw_gx; flt_val = flt_gx; break;   //Gx
        case 5: raw_val = raw_gy; flt_val = flt_gy; break;   //Gy
        case 6: raw_val = raw_gz; flt_val = flt_gz; break;   //Gz
        default: return;                  //无效轴号：不发送
    }

    uint8_t buf[12];
    float f = (float)raw_val;            //原始值转 float
    memcpy(&buf[0], &f, 4);              //ch0：原始值
    memcpy(&buf[4], &flt_val, 4);        //ch1：滤波后
    buf[8]  = 0x00;                      //帧尾 00 00 80 7F（JustFloat 结束标志）
    buf[9]  = 0x00;
    buf[10] = 0x80;
    buf[11] = 0x7F;
    NEUQ_Uart_write_data(UART_0, buf, 12);   //整帧一次性从 UART_0 发出
}

//数据显示
static void fmt_1dec(float v, char *out){
    int32_t tenths = (int32_t)(v * 10.0f);   //把值放大 10 倍取整，得到十分位表示
    char sign = ' ';
    if(tenths < 0){ sign = '-'; tenths = -tenths; }
    int32_t ip = tenths / 10;                //整数部分
    int32_t fp = tenths % 10;                //十分位
    snprintf(out, 16, "%c%3ld.%ld", sign, (long)ip, (long)fp);
}

//整数值 LSB”，用于六轴原始数据显示
static void draw_row(uint16_t y, const char *name, int32_t val){
    char line[32];
    snprintf(line, sizeof(line), "%s:%6d LSB", name, (int)val);
    NEUQ_LCD200_DisplayString(8, y, line);
}

//角度 deg”，用于姿态角显示
static void draw_angle_row(uint16_t y, const char *name, float deg){
    char num[16];
    char line[32];
    fmt_1dec(deg, num);
    snprintf(line, sizeof(line), "%s: %s deg", name, num);
    NEUQ_LCD200_DisplayString(8, y, line);
}

// 六轴数据显示页
static void draw_sixaxis_page(uint8_t full_redraw){
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    if(full_redraw){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(8, 8, "6-Axis Data");
    }
    draw_row(48,  "Ax", raw_ax);
    draw_row(80,  "Ay", raw_ay);
    draw_row(112, "Az", raw_az);
    draw_row(144, "Gx", raw_gx);
    draw_row(176, "Gy", raw_gy);
    draw_row(208, "Gz", raw_gz);
}

// 姿态解算显示页
static void draw_attitude_page(uint8_t full_redraw){
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    if(full_redraw){
        NEUQ_LCD200_Clear();
        NEUQ_LCD200_DisplayString(8, 8, "Attitude");
    }
    draw_angle_row(48,  "Roll",  roll);
    draw_angle_row(80,  "Pitch", pitch);
    draw_angle_row(112, "Yaw",   yaw);
}

//对外接口
void NEUQ_SixAxis_Init(void){
    NEUQ_Flash_init();                          //Flash 初始化
    flash_load();                               //从 Flash 加载上次保存的滤波系数

    NEUQ_MPU6050_init();                        //六轴传感器初始化

    NEUQ_Delay_ms(100);

    NEUQ_Uart_init(UART_0, UART_BAUD_1000000);  //VOFA+ 上传串口

    gyro_zero_calibrate();                      //陀螺仪 Z 轴零偏标定（上电需保持静止）

    display_tick = 0;
    last_page = MENU_NONE;                      //置为“无页面”，保证首次进入数据页立刻刷一次屏
}

// PIT_CH2(5ms) 中断回调
void NEUQ_SixAxis_TimeCallback(void){
    sample_count++;
}

// 主循环任务
void NEUQ_SixAxis_Task(void){
    //采样
    while(sample_done < sample_count){
        sample_done++;
        sample_process();                        //读传感器 + 六轴滤波 + 姿态解算

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
                uint8_t full = (page != last_page);  //切页
                if(page == MENU_DISPLAY){
                    draw_sixaxis_page(full);
                }else{
                    draw_attitude_page(full);
                }
            }
        }
        last_page = page;
    }

    //滤波系数持久化
    if(params.filter_coef != saved_filter_coef){
        flash_save();
    }
}
