/*
 * NEUQ_Setting.c
 * 系统参数的默认值、合法性检查、Flash读写
 */
#include "NEUQ_lib_include.h"
#include "NEUQ_Setting.h"

NEUQ_Setting_t g_setting;
uint8_t g_setting_dirty = 0;
uint8_t g_setting_valid = 0;

// FNV-1a 32bit，实现短分布也还行，用来判断Flash里的参数是不是一条完整记录
static uint32_t NEUQ_Setting_Hash(const uint8_t *buf, uint32_t len)
{
    uint32_t hash = 2166136261UL;
    for(uint32_t i = 0; i < len; i++)
    {
        hash ^= buf[i];
        hash *= 16777619UL;
    }
    return hash;
}

// 算校验和，算之前先把checksum字段清零
static uint32_t NEUQ_Setting_CalcChecksum(NEUQ_Setting_t *s)
{
    uint32_t save = s->checksum;
    s->checksum = 0;
    uint32_t hash = NEUQ_Setting_Hash((uint8_t *)s, (uint32_t)sizeof(NEUQ_Setting_t));
    s->checksum = save;
    return hash;
}

// 默认参数
// 加速度和角速度截止频率都给20Hz，既能压掉快速抖动，手摇的时候趋势也跟得上。
// Kp给0.5是Mahony的经典值，再大就会把加速度计的噪声也一起放大进去，静止时角度会抖。
// Ki给0是因为六轴没有磁力计，带了积分项航向会慢慢漂。
void NEUQ_Setting_Default(void)
{
    memset(&g_setting, 0, sizeof(g_setting));
    g_setting.magic       = NEUQ_SETTING_MAGIC;
    g_setting.version     = NEUQ_SETTING_VERSION;
    g_setting.size        = (uint16_t)sizeof(NEUQ_Setting_t);

    g_setting.wave_axis   = (uint8_t)NEUQ_AXIS_GYRO_X;
    g_setting.wave_enable = 0;
    g_setting.wave_rate   = (uint8_t)NEUQ_WAVE_RATE_100;

    g_setting.acc_lpf_hz  = 20.0f;
    g_setting.gyro_lpf_hz = 20.0f;
    g_setting.att_kp      = 0.5f;
    g_setting.att_ki      = 0.0f;

    g_setting.checksum    = NEUQ_Setting_CalcChecksum(&g_setting);
    g_setting_dirty       = 0;
}

// 把越界值钳到合法范围
void NEUQ_Setting_Sanitize(NEUQ_Setting_t *s)
{
    if(s->wave_axis   >= (uint8_t)NEUQ_AXIS_NUM)        s->wave_axis   = (uint8_t)NEUQ_AXIS_GYRO_X;
    if(s->wave_enable >  1U)                            s->wave_enable = 0;
    if(s->wave_rate   >= (uint8_t)NEUQ_WAVE_RATE_NUM)   s->wave_rate   = (uint8_t)NEUQ_WAVE_RATE_100;

    // NaN比较恒为假，所以先判"不小于"再判"不大于"，这样NaN也会被挡住
    if(!(s->acc_lpf_hz  >= NEUQ_SETTING_FC_MIN) || !(s->acc_lpf_hz  <= NEUQ_SETTING_FC_MAX))  s->acc_lpf_hz  = 20.0f;
    if(!(s->gyro_lpf_hz >= NEUQ_SETTING_FC_MIN) || !(s->gyro_lpf_hz <= NEUQ_SETTING_FC_MAX))  s->gyro_lpf_hz = 20.0f;
    if(!(s->att_kp      >= NEUQ_SETTING_KP_MIN) || !(s->att_kp      <= NEUQ_SETTING_KP_MAX))  s->att_kp      = 1.0f;
    if(!(s->att_ki      >= NEUQ_SETTING_KI_MIN) || !(s->att_ki      <= NEUQ_SETTING_KI_MAX))  s->att_ki      = 0.0f;
}

// 从Flash读参数
// 先读到临时结构体，全部校验通过了再赋给g_setting，
// 这样就算Flash里是0xFF或者半截数据也不会污染正在用的参数。
void NEUQ_Setting_Load(void)
{
    NEUQ_Setting_t tmp;
    uint8_t ok = 0;

    memset(&tmp, 0, sizeof(tmp));

    if(NEUQ_Flash_init() == NEUQ_FLASH_OK)
    {
        if(NEUQ_Flash_read(NEUQ_SETTING_FLASH_ADDR, (uint8_t *)&tmp, sizeof(tmp)) == NEUQ_FLASH_OK)
        {
            if(tmp.magic   == NEUQ_SETTING_MAGIC  &&
               tmp.version == NEUQ_SETTING_VERSION &&
               tmp.size    == (uint16_t)sizeof(NEUQ_Setting_t))
            {
                uint32_t calc = NEUQ_Setting_CalcChecksum(&tmp);
                if(calc == tmp.checksum)
                {
                    ok = 1;
                }
            }
        }
    }

    if(ok)
    {
        g_setting = tmp;
        NEUQ_Setting_Sanitize(&g_setting);
        g_setting_valid = 1;
        g_setting_dirty = 0;
    }
    else
    {
        // 没有有效记录，第一次上电或者换过Flash都是走这里
        NEUQ_Setting_Default();
        g_setting_valid = 0;
        g_setting_dirty = 0;
    }
}

// 写Flash。擦除、写入、回读校验都由NEUQ_Flash_save做
uint8_t NEUQ_Setting_Save(void)
{
    NEUQ_Setting_t tmp = g_setting;

    tmp.magic    = NEUQ_SETTING_MAGIC;
    tmp.version  = NEUQ_SETTING_VERSION;
    tmp.size     = (uint16_t)sizeof(NEUQ_Setting_t);
    tmp.checksum = NEUQ_Setting_CalcChecksum(&tmp);

    if(NEUQ_Flash_save(NEUQ_SETTING_FLASH_ADDR, (const uint8_t *)&tmp, (uint32_t)sizeof(tmp)) != NEUQ_FLASH_OK)
    {
        return NEUQ_FLASH_ERROR;
    }

    g_setting       = tmp;      // 保存成功了就以落盘的数据为准
    g_setting_valid = 1;
    g_setting_dirty = 0;
    return NEUQ_FLASH_OK;
}

const char *NEUQ_Setting_AxisName(uint8_t axis)
{
    static const char *name[NEUQ_AXIS_NUM] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};
    if(axis >= (uint8_t)NEUQ_AXIS_NUM) axis = 0;
    return name[axis];
}

const char *NEUQ_Setting_AxisText(uint8_t axis)
{
    static const char *text[NEUQ_AXIS_NUM] = {"accel X", "accel Y", "accel Z",
                                              "gyro  X", "gyro  Y", "gyro  Z"};
    if(axis >= (uint8_t)NEUQ_AXIS_NUM) axis = 0;
    return text[axis];
}

const char *NEUQ_Setting_AxisUnit(uint8_t axis)
{
    return (axis < (uint8_t)NEUQ_AXIS_GYRO_X) ? "g" : "dps";
}

uint16_t NEUQ_Setting_WaveRateHz(uint8_t rate)
{
    static const uint16_t hz[NEUQ_WAVE_RATE_NUM] = {50, 100, 200, 500};
    if(rate >= (uint8_t)NEUQ_WAVE_RATE_NUM) rate = (uint8_t)NEUQ_WAVE_RATE_100;
    return hz[rate];
}
