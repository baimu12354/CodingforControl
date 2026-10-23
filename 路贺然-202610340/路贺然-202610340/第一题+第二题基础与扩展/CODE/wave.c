#include "NEUQ_lib_include.h"
#include "wave.h"
//===========================断电保存==================================
#ifndef NEUQ_MENU_PARAM_DEFINED
#define NEUQ_MENU_PARAM_DEFINED

typedef struct {
    int32_t speed;
    int32_t accel;
    int32_t servo_center;
    int32_t servo_limit;
    int32_t filter_axis;
    int32_t filter_enable;
    int32_t filter_param;
} NEUQ_MENU_PARAM;

#endif

extern NEUQ_MENU_PARAM params;
//============================================================================
static float s_raw[6];
static float s_flt[6];
static float s_prev[6];

volatile uint8_t      g_wave_save_req=0;
#define WAVE_UART     UART_1
#define WAVE_BAUD     UART_BAUD_115200
#define WAVE_MAGIC    0xAA55AA55UL
//==============================、、恢复参数、、===================================================
typedef struct{
    uint32_t magic;         //魔术字
    int32_t filter_axis;    //0-5
    int32_t filter_enable;  //0-1
    int32_t filter_param;   //1-100
}WaveFlash_t;
//初始化
void Wave_Init(void){
    //初始上传接口
    NEUQ_Uart_init(WAVE_UART,WAVE_BAUD);
    //flash回复参数
    WaveFlash_t d;
    //读取失败
    if(NEUQ_Flash_read_sector(NEUQ_FLASH_LAST_SECTOR_ID,
                            (uint8_t*)&d,
                            sizeof(d))!=NEUQ_FLASH_OK){
                                return;
                            }
    //验证magic
    if(d.magic!=WAVE_MAGIC){
        return;
    }
    if(d.filter_axis<0||d.filter_axis>5)return;
    if(d.filter_enable<0||d.filter_enable>1)return;
    if(d.filter_param<1||d.filter_param>100)return;
    params.filter_axis   = d.filter_axis;
    params.filter_enable = d.filter_enable;
    params.filter_param  = d.filter_param;
}
    void Wave_Tick(void){

    s_raw[0] = (float)MPU6050_acc_transition(mpu6050_acc_x)*10;
    s_raw[1] = (float)MPU6050_acc_transition(mpu6050_acc_y)*10;   
    s_raw[2] = (float)MPU6050_acc_transition(mpu6050_acc_z)*10;    
    s_raw[3] = (float)MPU6050_gyro_transition(mpu6050_gyro_x);   
    s_raw[4] = (float)MPU6050_gyro_transition(mpu6050_gyro_y);   
    s_raw[5] = (float)MPU6050_gyro_transition(mpu6050_gyro_z);  
//================一阶低通===============
    float a = params.filter_param / 100.0f;
    float b = 1.0f - a;
    for (int i = 0; i < 6; i++) {
    s_flt[i]  = a * s_raw[i] + b * s_prev[i];
    s_prev[i] = s_flt[i];}

//============打包上传===================
uint8_t ax=(uint8_t)params.filter_axis;
if(params.filter_enable&&ax<6){

    float r=s_raw[ax];
    float f=s_flt[ax];
    printf("%f,%f\n",r,f);
}
}
//============flash save===========================
void Wave_FlashSave(void){
    WaveFlash_t d;
    d.magic         =WAVE_MAGIC;
    d.filter_axis   =params.filter_axis;
    d.filter_enable =params.filter_enable;
    d.filter_param  =params.filter_param;
    NEUQ_Flash_write_sector(NEUQ_FLASH_LAST_SECTOR_ID,
                            (const uint8_t*)&d,
                            sizeof(d));
}
volatile uint8_t g_wave_tick_req = 0;
void PIT_ISR(void){
    g_wave_tick_req = 1;
}