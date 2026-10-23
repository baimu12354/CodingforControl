#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "config.h"

typedef enum { AX_AX,AX_AY,AX_AZ,AX_GX,AX_GY,AX_GZ,AX_NUM } Axis_Id;
typedef enum { PAGE_MAIN,PAGE_DATA,PAGE_SETTING,PAGE_ATTITUDE } Page_Id;
typedef enum { KEY_UP,KEY_DOWN,KEY_ENTER,KEY_BACK,KEY_NUM } Key_Id;

typedef struct { uint32_t magic; uint8_t filter_percent, send_axis, send_enable, reserved; } Param_t;

extern volatile uint32_t time_tick;
extern Param_t param;
extern float raw_axis[AX_NUM], filt_axis[AX_NUM];
extern float att_roll, att_pitch, att_yaw;
extern Page_Id page;
extern uint32_t last_save_tick;

#endif
