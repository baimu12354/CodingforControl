#ifndef _MENU_
#define _MENU_
#include "NEUQ_Key.h"
#include "NEUQ_lib_include.h"
typedef enum {
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_DATA,
    MEUN_WAVE,
    MENU_DECODER,
    WAVE_SEND_X,
    WAVE_SEND_Y,
    WAVE_SEND_Z,
    WAVE_SEND_ONOFF,
    WAVE_SEND_FLI_A,/*XIAN只留一个参数接口*/
    WAVE_SEND_SAVE,
//    DECODER_A,
    DECODER_RESET,
    MENU_COUNT,
}MENUID;
typedef struct{
    int32_t flit;
    int32_t  wavestate;
    int32_t deca;
    int32_t  xyzchoose;
}menuparamas;
extern menuparamas paramas;
typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float axpro;
    float aypro;
    float azpro;
    float gxpro;
    float gypro;
    float gzpro;
    float sdalite;
    float sdapro;
    float sdglite;
    float sdgpro;
}imudata;
extern imudata imdt;
typedef struct menu
{
    const char *zhushi;
    MENUID parent;
    int32_t *value;
    int32_t step;
    int32_t min;
    int32_t max;
}menuconfig;
 
typedef struct menode{
    const menuconfig *info;
    struct menode *parent;
    struct menode *child;
    struct menode *last;
    struct menode *next;
}menunoed;
extern menunoed nodes[MENU_COUNT];
void buildlink (void);
uint8_t menuupdate (KEY_EVENT_ENUM key);
void dataget    (void);
void dataall (void );
void datachoose(int8_t);
void send    (void);
void shortage (void);
void mainshow (void);
void datashow (void);
void decodeshow (void);
void menutask (void);
extern volatile uint8_t datashowflag ;
extern volatile uint8_t datashowflag2 ;
extern volatile uint8_t decodeshowflag ;
extern volatile uint8_t decodeshowflag2 ;
extern volatile uint8_t sendflag ;
extern volatile uint8_t reflashflag ;
void decoderreset (int32_t samples);
void decoderresetall (void);        /* [新] 运行中重置：重标零偏 + 姿态归零 */

void mahonyupdate (float ax,float ay,float az,float gx,float gy,float gz,float dt);   /* [改] 补 dt 参数 */
void mahonyreset (void);            /* [新] 清 Mahony 积分项 */
void Quat_To_Euler(void);
typedef struct 
{float x,y,z,w;}siyuan;
typedef struct {
    float roll;
    float pitch;
    float yaw;
}sanyuan;
typedef struct 
{float in;
 float out;
 float a;
}MENU_LPF1;
void  MENU_LPF1_Init(MENU_LPF1 *lp, float alpha);
void  MENU_LPF1_SetAlpha(MENU_LPF1 *lp, float alpha);
float MENU_LPF1_Calc(MENU_LPF1 *lp, float ru);

extern sanyuan e;
extern siyuan q;
#endif