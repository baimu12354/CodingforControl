·························/*********************************************************************************************************************
* NEUQ Library ����������ѧ�ػʵ���У ��Դ�⣩
* �ļ���NEUQ_Main   ƽ̨��STM32F407VE   ������MDK-ARM v5.38
* ���ܣ�MPU6050 ����ɼ�?+ ��ͨ�˲� + ��̬���� + LCD �˵� + VOFA ���� + Flash ����
* ���ߣ�MPU6050->Hard_IIC3(SCL:PA8 SDA:PC9)��LCD->SPI3��VOFA->UART0(PA9/PA10)
* ������PE10=��  PE8=��  PE7=ȷ��  PE9=����
*********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "Attitude.h"

// ===== 4 �����ذ��� =====
#define KEY_UP        PE10
#define KEY_DOWN      PE8
#define KEY_FORWARD   PE7
#define KEY_BACK      PE9
#define KEY_NONE      0xFF

// ===== ҳ���� =====
#define PAGE_HOME      0
#define PAGE_6AXIS     1
#define PAGE_FILTER    2
#define PAGE_ATTITUDE  3

#define FLASH_SECTOR   NEUQ_FLASH_LAST_SECTOR_ID   // W25Q16 ���һ��?4KB ����

// ===== ȫ������ =====
float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;   // ԭʼֵ
float ax_flt, ay_flt, az_flt, gx_flt, gy_flt, gz_flt;   // �˲�ֵ
float   flt_alpha = 0.05f;   // һ�׵�ͨϵ��
uint8_t vofa_en   = 0;       // VOFA ����
uint8_t vofa_axis = 0;       // VOFA �� 0..5 = AX AY AZ GX GY GZ

static uint8_t menu_dirty = 0;   // menu needs redraw
static uint8_t page = PAGE_HOME;   // ��ǰҳ��
static uint8_t sel  = 0;
static uint8_t home_sel = 0;           // �˵����?
// ===== gz dynamic bias (on top of Attitude_Calib static calib) =====
#define ENABLE_DYN_GZ_BIAS   1        // 1=on 0=off
#define DYN_GZ_STILL_THRESH  1.5f     // still threshold deg/s
#define DYN_GZ_SAMPLE_NUM    100      // still sample count

uint8_t first_sample[6] = {1,1,1,1,1,1};
#if ENABLE_DYN_GZ_BIAS
static float    dyn_gz_bias = 0.0f;   // runtime gz bias deg/s
static float    dyn_gz_sum  = 0.0f;
static uint16_t dyn_gz_cnt  = 0;
#endif
// ===== ������ =====
static const char *axis_name(uint8_t a)
{
    static const char *n[6] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};
    return (a < 6) ? n[a] : "??";
}

// ===== һ�׵�ͨ =====
static float lowpass(float in, float out, float a)
{
    return out + a * (in - out);
}

// ===== Flash ������д =====
static void param_load(void)
{
    float t = 0.0f;
    if (NEUQ_Flash_read_sector(FLASH_SECTOR, (uint8_t *)&t, sizeof(t)) == NEUQ_FLASH_OK
        && t > 0.0f && t <= 1.0f)
        flt_alpha = t;
}

static void param_save(void)
{
    NEUQ_Flash_write_sector(FLASH_SECTOR, (uint8_t *)&flt_alpha, sizeof(flt_alpha));
}

// ===== VOFA��ֻ��ѡ���� ԭʼֵ,�˲�ֵ =====
static void vofa_send(void)
{
    const float *r[6] = {&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw};
    const float *f[6] = {&ax_flt, &ay_flt, &az_flt, &gx_flt, &gy_flt, &gz_flt};
    if (!vofa_en || vofa_axis > 5) return;
    printf("%.4f,%.4f\r\n", *r[vofa_axis], *f[vofa_axis]);
}

// ===== ������������ 0..3���ް������� KEY_NONE =====
static uint8_t key_read(void)
{
    if (NEUQ_GPIO_read(KEY_UP)      == GPIO_Low) return 0;
    if (NEUQ_GPIO_read(KEY_DOWN)    == GPIO_Low) return 1;
    if (NEUQ_GPIO_read(KEY_FORWARD) == GPIO_Low) return 2;
    if (NEUQ_GPIO_read(KEY_BACK)    == GPIO_Low) return 3;
    return KEY_NONE;
}

// ===== �˵������߼� =====
static void menu_key(uint8_t k)
{
    if (page == PAGE_HOME) {
        if (k == 0)            sel = (sel + 2) % 3;   // ����
        else if (k == 1)       sel = (sel + 1) % 3;   // ����
        else if (k == 2) { home_sel = sel; page = sel + 1; sel = 0; } // ����
    }
    else if (page == PAGE_FILTER) {
        if (k == 0)            sel = (sel + 4) % 5;
        else if (k == 1)       sel = (sel + 1) % 5;
        else if (k == 2) {
            switch (sel) {
                case 0: flt_alpha += 0.05f; if (flt_alpha > 1.0f) flt_alpha = 0.05f; break;
                case 1: vofa_axis = (vofa_axis + 1) % 6; break;
                case 2: vofa_en = !vofa_en; break;
                case 3: param_save(); break;
                case 4: page = PAGE_HOME; sel = home_sel; break;
            }
        }
        else if (k == 3) { page = PAGE_HOME; sel = home_sel; }
    }
    else {                       // Ҷ��ҳ��������ҳ
        if (k == 3) { page = PAGE_HOME; sel = home_sel; }
    }
}

// ===== �˵�ҳ��HOME / FILTER���������� =====
static void draw_menu(void)
{
    char l0[16], l1[16], l2[16];
    char *items[5];
    uint8_t n;
    int i, y;

    NEUQ_LCD200_Clear();
    if (page == PAGE_HOME) {
        NEUQ_LCD200_DisplayString(0, 0, "HOME MENU");
        items[0] = "6AXIS DATA";
        items[1] = "FILTER & VOFA";
        items[2] = "ATTITUDE";
        n = 3;
    } else {
        NEUQ_LCD200_DisplayString(0, 0, "FILTER & VOFA");
        snprintf(l0, sizeof(l0), "alpha %.2f", flt_alpha);
        snprintf(l1, sizeof(l1), "axis %s", axis_name(vofa_axis));
        snprintf(l2, sizeof(l2), "upload %s", vofa_en ? "ON" : "OFF");
        items[0] = l0; items[1] = l1; items[2] = l2;
        items[3] = "SAVE"; items[4] = "BACK";
        n = 5;
    }
    for (i = 0, y = 24; i < n; i++, y += 20) {
        NEUQ_LCD200_DisplayString(0,  y, (i == sel) ? ">" : " ");
        NEUQ_LCD200_DisplayString(20, y, items[i]);
    }
}

// ===== Ҷ��ҳ������ / ��̬����̬��ǩ =====
static void leaf_static(void)
{
    NEUQ_LCD200_Clear();
    if (page == PAGE_6AXIS) {
        NEUQ_LCD200_DisplayString(0, 0, "6AXIS (FILTERED)");
        NEUQ_LCD200_DisplayString(0,  24, "AX:");
        NEUQ_LCD200_DisplayString(0,  44, "AY:");
        NEUQ_LCD200_DisplayString(0,  64, "AZ:");
        NEUQ_LCD200_DisplayString(0,  84, "GX:");
        NEUQ_LCD200_DisplayString(0, 104, "GY:");
        NEUQ_LCD200_DisplayString(0, 124, "GZ:");
    } else {
        NEUQ_LCD200_DisplayString(0, 0, "ATTITUDE");
        NEUQ_LCD200_DisplayString(0,  40, "ROLL:");
        NEUQ_LCD200_DisplayString(0,  80, "PITCH:");
        NEUQ_LCD200_DisplayString(0, 120, "YAW:");
    }
    NEUQ_LCD200_DisplayString(0, 208, "BACK=KEY_BACK");
}

// ===== Ҷ��ҳ��ֵˢ�£�������������˸�� =====
static void leaf_values(void)
{
    char b[16];
    if (page == PAGE_6AXIS) {
        snprintf(b, sizeof(b), "%8.2f", ax_flt); NEUQ_LCD200_DisplayString(80,  24, b);
        snprintf(b, sizeof(b), "%8.2f", ay_flt); NEUQ_LCD200_DisplayString(80,  44, b);
        snprintf(b, sizeof(b), "%8.2f", az_flt); NEUQ_LCD200_DisplayString(80,  64, b);
        snprintf(b, sizeof(b), "%8.2f", gx_flt); NEUQ_LCD200_DisplayString(80,  84, b);
        snprintf(b, sizeof(b), "%8.2f", gy_flt); NEUQ_LCD200_DisplayString(80, 104, b);
        snprintf(b, sizeof(b), "%8.2f", gz_flt); NEUQ_LCD200_DisplayString(80, 124, b);
    } else {
        snprintf(b, sizeof(b), "%8.1f", Attitude_GetRoll());  NEUQ_LCD200_DisplayString(80,  40, b);
        snprintf(b, sizeof(b), "%8.1f", Attitude_GetPitch()); NEUQ_LCD200_DisplayString(80,  80, b);
        snprintf(b, sizeof(b), "%8.1f", Attitude_GetYaw());   NEUQ_LCD200_DisplayString(80, 120, b);
    }
}

// ===== ���������� HAL main.c ���ã� =====
void User_Main(void)
{
    uint16_t last_us, last_draw, now;
    uint8_t  last_key, prev_page, k;

    // �����ʼ��?
    NEUQ_Delay_init();      // ʹ�� DWT
    NEUQ_Timer_init();      // ʹ�� TIM13 ��ʱ��
    NEUQ_Debug_init();      // UART0 + printf �ض���
    NEUQ_LCD200_init();
    NEUQ_MPU6050_init();
    NEUQ_Flash_init();

    NEUQ_GPIO_init(KEY_UP,      GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_FORWARD, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK,    GPIO_Mode_Input_PullUp, GPIO_High);

    param_load();

    last_us   = (uint16_t)NEUQ_Get_Timer_Value_us();
    last_draw = last_us;
    last_key  = KEY_NONE;
    prev_page = 0xFF;

    Attitude_Calib();   //上电执行陀螺仪零偏校准
    last_us   = (uint16_t)NEUQ_Get_Timer_Value_us();
    draw_menu();   // �ϵ���ʾ��ҳ

    while (1) {
        now = (uint16_t)NEUQ_Get_Timer_Value_us();

        // ÿ 5ms ����һ�Σ�200Hz��
        if ((uint16_t)(now - last_us) >= 5000) {
            last_us = now;
            NEUQ_MPU6050_read_data();
            ax_raw = MPU6050_acc_transition(mpu6050_acc_x);
            ay_raw = MPU6050_acc_transition(mpu6050_acc_y);
            az_raw = MPU6050_acc_transition(mpu6050_acc_z);
            gx_raw = MPU6050_gyro_transition(mpu6050_gyro_x);
            gy_raw = MPU6050_gyro_transition(mpu6050_gyro_y);
            gz_raw = MPU6050_gyro_transition(mpu6050_gyro_z);

            //AX 0 AY1 AZ2 GX3 GY4 GZ5
    if(first_sample[0]){ ax_flt = ax_raw; first_sample[0]=0; }
   else ax_flt = lowpass(ax_raw, ax_flt, flt_alpha);

    if(first_sample[1]){ ay_flt = ay_raw; first_sample[1]=0; }
   else ay_flt = lowpass(ay_raw, ay_flt, flt_alpha);

    if(first_sample[2]){ az_flt = az_raw; first_sample[2]=0; }
   else az_flt = lowpass(az_raw, az_flt, flt_alpha);

   if(first_sample[3]){ gx_flt = gx_raw; first_sample[3]=0; }
  else gx_flt = lowpass(gx_raw, gx_flt, flt_alpha);
 
   if(first_sample[4]){ gy_flt = gy_raw; first_sample[4]=0; }
  else gy_flt = lowpass(gy_raw, gy_flt, flt_alpha);

  if(first_sample[5]){ gz_flt = gz_raw; first_sample[5]=0; }
  else gz_flt = lowpass(gz_raw, gz_flt, flt_alpha);

#if ENABLE_DYN_GZ_BIAS
            // gz dynamic bias: collect only when still, reset on move
            if (gz_flt < DYN_GZ_STILL_THRESH && gz_flt > -DYN_GZ_STILL_THRESH) {
                dyn_gz_sum += gz_flt;
                if (++dyn_gz_cnt >= DYN_GZ_SAMPLE_NUM) {
                    dyn_gz_bias = dyn_gz_sum / (float)dyn_gz_cnt;
                    dyn_gz_sum  = 0.0f;
                    dyn_gz_cnt  = 0;
                }
            } else {
                dyn_gz_sum = 0.0f;
                dyn_gz_cnt = 0;
            }
#endif


            Attitude_Update(gx_flt, gy_flt, gz_flt
#if ENABLE_DYN_GZ_BIAS
                - dyn_gz_bias
#endif
                , ax_flt, ay_flt, az_flt);
            vofa_send();
        }

        // ���������ش��� + 20ms ����
        k = key_read();
        if (k != last_key) {
            NEUQ_Delay_ms(20);
            k = key_read();
            if (k != last_key) {
                if (k != KEY_NONE) { menu_key(k); menu_dirty = 1; }
                last_key = k;
            }
        }

        // ҳ���л���Ҷ��ҳ�ػ���̬��ǩ
        if (page != prev_page) {
            prev_page = page;
            if (page == PAGE_6AXIS || page == PAGE_ATTITUDE) leaf_static();
        }

        // ÿ 20ms �ػ�һ��
        if ((uint16_t)(now - last_draw) >= 20000) {
            last_draw = now;
            if (page == PAGE_6AXIS || page == PAGE_ATTITUDE) leaf_values();
        }

        // redraw menu only when dirty (no per-20ms full clear)
        if (menu_dirty) {
            menu_dirty = 0;
            if (page == PAGE_HOME || page == PAGE_FILTER) draw_menu();
        }
    }
}
