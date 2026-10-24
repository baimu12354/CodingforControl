/* board_config.h - 引脚与参数配置 */

#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

/* 按键：上拉输入，按下为低 */
#define T2_KEY_UP_PIN           (PE9)
#define T2_KEY_DOWN_PIN         (PE7)
#define T2_KEY_ENTER_PIN        (PE8)
#define T2_KEY_BACK_PIN         (PE10)

#define T2_KEY_SHORT_MIN_TICKS  (2U)    /* 20 ms */
#define T2_KEY_LONG_TICKS       (50U)   /* 500 ms */

#define T2_TICK_MS              (1U)    /* PIT_CH1(TIM6) 基础节拍 1 ms */
#define T2_KEY_SCAN_TICKS       (10U)   /* 每 10 ms 扫描一次按键 */
#define T2_SAMPLE_TICKS         (5U)    /* 每 5 ms 允许一次采样（200 Hz 目标） */
#define T2_UPLOAD_TICKS         (50U)   /* 每 50 ms 上传一帧（20 Hz） */
#define T2_UI_TICKS             (100U)  /* 每 100 ms 刷新一次数值显示 */

/* MPU6050 地址由 AD0 决定（0x68/0x69），本工程上电自动探测 */
#define T2_MPU_ADDR_PRIMARY     (0x68U)
#define T2_MPU_ADDR_ALTERNATE   (0x69U)

/* 量程与库的默认配置保持一致：加速度 ±8g，陀螺仪 ±2000°/s */
#define T2_ACC_LSB_PER_G        (4096.0f)
#define T2_GYRO_LSB_PER_DPS     (16.4f)

/* 六路一阶低通系数：0.05~0.95，步长 0.05 */
#define T2_FILTER_DEFAULT_ALPHA (0.30f)
#define T2_FILTER_ALPHA_MIN     (0.05f)
#define T2_FILTER_ALPHA_MAX     (0.95f)
#define T2_FILTER_ALPHA_STEP    (0.05f)

/* 直接用字面量 511，避免 board_config.h 依赖库头文件的包含顺序 */
#define T2_PARAM_SECTOR         (511U)
#define T2_PARAM_MAGIC          (0x4E455132UL)      /* "NEQ2" */

/* 芯片轴 → 物理轴：0=X 1=Y 2=Z；方向不对改 SIGN */
#define T2_ATT_ROLL_AXIS        (0)
#define T2_ATT_ROLL_SIGN        (+1.0f)
#define T2_ATT_PITCH_AXIS       (1)
#define T2_ATT_PITCH_SIGN       (+1.0f)
#define T2_ATT_YAW_AXIS         (2)
#define T2_ATT_YAW_SIGN         (+1.0f)

/* 加速度计修正强度，1.0 约 1 s 收敛；越大越信加速度计 */
#define T2_ATT_ACC_TRUST        (1.0f)

/* 姿态模块内部对加速度计做的一次固定低通，专门压噪声。
 * 写死在这里，不受菜单里滤波系数的影响——挑战任务的性能不该被基础设置绑架。 */
#define T2_ATT_ACC_LPF_ALPHA    (0.20f)

/* 零偏校准：上电后先给 2 秒把板子放平，再采 2 秒零偏。全程不阻塞。 */
#define T2_ATT_CAL_PREPARE_MS   (2000U)
#define T2_ATT_CAL_SAMPLE_MS    (2000U)
/* 采样期间角速度模长超过这个值（°/s）就认为板子在动，清零重来 */
#define T2_ATT_CAL_MOTION_DPS   (4.0f)

/* 单次积分的步长上限（毫秒）。主循环刷屏时采样间隔会拉长，
 * 不设上限的话一次积分会跳一大步，屏幕上就是角度突然蹦一下。 */
#define T2_ATT_MAX_DT_MS        (50U)

#endif /* _BOARD_CONFIG_H_ */
