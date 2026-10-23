/*
 * NEUQ_Setting.h
 * 系统参数（选轴 / 上传开关 / 上传速率 / 滤波截止频率 / 姿态增益）的定义和掉电保存
 *
 * 几点说明：
 *   1. 整个程序只有一份参数实体 g_setting，菜单显示的是它，后台用的也是它，
 *      所以"页面显示"和"实际在跑的状态"不会出现不一致。
 *   2. 改完参数只是把 g_setting_dirty 置1，只有点了Save to Flash才真正写盘。
 *   3. Flash里存的是"魔数+版本+长度+校验和"，重启后校验不过就用默认值。
 */
#ifndef _NEUQ_SETTING_H_
#define _NEUQ_SETTING_H_

#include "NEUQ_lib_include.h"

// Flash存放位置
// 板载W25Q16一共512个4KB扇区(0~511)，NEUQ库的掉电保存例程用了最后一个(511)，
// 这里用510号，两边不会打架。
#define NEUQ_SETTING_FLASH_ADDR       (0x1FE000UL)      // 510号扇区起始地址
#define NEUQ_SETTING_MAGIC            (0x4E515357UL)    // "NQSW"
#define NEUQ_SETTING_VERSION          (0x0101U)         // 结构或默认值改了就把这个加一

// 参数范围，菜单编辑和Flash合法性检查共用，免得越界值被写进去
#define NEUQ_SETTING_FC_MIN           (1.0f)            // 滤波截止频率下限 Hz
#define NEUQ_SETTING_FC_MAX           (100.0f)          // 上限 Hz
#define NEUQ_SETTING_KP_MIN           (0.0f)
#define NEUQ_SETTING_KP_MAX           (10.0f)
#define NEUQ_SETTING_KI_MIN           (0.0f)
#define NEUQ_SETTING_KI_MAX           (0.5f)

// 选轴，上传时任意时刻只能选一个
typedef enum{
    NEUQ_AXIS_ACC_X = 0,
    NEUQ_AXIS_ACC_Y,
    NEUQ_AXIS_ACC_Z,
    NEUQ_AXIS_GYRO_X,
    NEUQ_AXIS_GYRO_Y,
    NEUQ_AXIS_GYRO_Z,
    NEUQ_AXIS_NUM               // = 6
} NEUQ_AXIS_ENUM;

// 上传速率档位
typedef enum{
    NEUQ_WAVE_RATE_50 = 0,
    NEUQ_WAVE_RATE_100,
    NEUQ_WAVE_RATE_200,
    NEUQ_WAVE_RATE_500,
    NEUQ_WAVE_RATE_NUM
} NEUQ_WAVE_RATE_ENUM;

// 需要掉电保存的参数
// 结构体长度固定、没有隐式填充，所以可以直接按字节写进Flash
typedef struct{
    uint32_t magic;             // NEUQ_SETTING_MAGIC
    uint16_t version;           // NEUQ_SETTING_VERSION
    uint16_t size;              // sizeof(NEUQ_Setting_t)
    uint32_t checksum;          // 这个字段置0后对整个结构做FNV-1a

    uint8_t  wave_axis;         // NEUQ_AXIS_ENUM，当前上传的轴
    uint8_t  wave_enable;       // 0=停 1=发
    uint8_t  wave_rate;         // NEUQ_WAVE_RATE_ENUM
    uint8_t  reserved8;

    float    acc_lpf_hz;        // 加速度三轴低通截止频率
    float    gyro_lpf_hz;       // 角速度三轴低通截止频率
    float    att_kp;
    float    att_ki;

    uint32_t reserved32[3];     // 留着以后加参数，不用改版本号
} NEUQ_Setting_t;

extern NEUQ_Setting_t g_setting;      // 当前生效参数
extern uint8_t  g_setting_dirty;      // 1 = 有改动还没保存
extern uint8_t  g_setting_valid;      // 1 = 本次上电是从Flash读的，0 = 用的默认值

void NEUQ_Setting_Default(void);                    // 恢复默认值
void NEUQ_Setting_Load(void);                       // 上电读Flash，读不到就用默认值
uint8_t NEUQ_Setting_Save(void);                    // 写Flash，返回NEUQ_FLASH_OK/ERROR
void NEUQ_Setting_Sanitize(NEUQ_Setting_t *s);      // 把越界值钳回合法范围

// 显示用的小工具
const char *NEUQ_Setting_AxisName(uint8_t axis);    // "AX" / "AY" / ... / "GZ"
const char *NEUQ_Setting_AxisText(uint8_t axis);    // "accel X" / "gyro X" ...
const char *NEUQ_Setting_AxisUnit(uint8_t axis);    // "g" / "dps"
uint16_t    NEUQ_Setting_WaveRateHz(uint8_t rate);  // 速率枚举转Hz

#endif /* _NEUQ_SETTING_H_ */
