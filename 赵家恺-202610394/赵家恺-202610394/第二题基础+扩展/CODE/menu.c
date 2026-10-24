#include "NEUQ_Key.h"
#include "menu.h"
#include <stdint.h>
#include <math.h>   
menuparamas paramas;
menunoed nodes[MENU_COUNT];
void shortage (void);
void datachoose(int8_t);
volatile uint8_t datashowflag = 0;
volatile uint8_t datashowflag2 = 0;
volatile uint8_t decodeshowflag = 0;
volatile uint8_t decodeshowflag2 = 0;
volatile uint8_t sendflag = 0;
volatile uint8_t reflashflag = 0;
    static menunoed *selected;
    static menunoed *page;
    static  int8_t   editing;
    static int32_t  editingvalue;
void MENU_LPF1_Init(MENU_LPF1 *lp, float alpha)
{
    if (lp == NULL) return;
    MENU_LPF1_SetAlpha(lp, alpha);
    lp->in  = 0.0f;
    lp->out = 0.0f;
}
void MENU_LPF1_SetAlpha(MENU_LPF1 *lp, float alpha)
{
    if (lp == NULL) return;

    if (alpha < 0.0f)  alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;  

    lp->a = alpha;
}

float MENU_LPF1_Calc(MENU_LPF1 *lp, float ru)
{
    if (lp == NULL) return ru;

    lp->in  = ru;
    lp->out = lp->out * lp->a + lp->in * (1.0f - lp->a);
    return lp->out;
}
static const menuconfig table [MENU_COUNT]={
    /* zhushi, parent, value, step, min, max */
    [MENU_MAIN]       = {"menumain",   MENU_NONE,  NULL, 0, 0,    0},
    [MENU_DATA]       = {"menudata",   MENU_MAIN,  NULL, 0, 0,    0},
    [MEUN_WAVE]       = {"menuwave",   MENU_MAIN,  NULL, 0, 0,    0},
    [MENU_DECODER]    = {"menudecoder",MENU_MAIN,  NULL, 0, 0,    0},
    [WAVE_SEND_X]     = {"sendx",      MEUN_WAVE, NULL, 0, 0, 0},
    [WAVE_SEND_Y]     = {"sendy",      MEUN_WAVE, NULL, 0, 0, 0},
    [WAVE_SEND_Z]     = {"sendz",      MEUN_WAVE, NULL, 0, 0, 0},
    [WAVE_SEND_ONOFF] = {"sendonoff",  MEUN_WAVE, &paramas.wavestate, 1, 0,    1},
    [WAVE_SEND_FLI_A] = {"sendflia",   MEUN_WAVE, &paramas.flit, 1, 0, 100},
    [WAVE_SEND_SAVE]  = {"sendsave",   MEUN_WAVE, NULL, 0, 0,    0},
//   [DECODER_A]       = {"decodera",   MENU_DECODER,&paramas.deca,1,-100,100},
    [DECODER_RESET]   = {"decoderreset",MENU_DECODER,NULL, 0, 0, 0},
};
void buildlink (void){
    for (int8_t i = 0; i < MENU_COUNT; i++)
    {
        nodes[i].info = &table[i];
        nodes[i].next = NULL;
        nodes[i].last = NULL;
        nodes[i].parent = NULL;
        nodes[i].child = NULL;       
    };
    for (int8_t i = 0; i < MENU_COUNT; i++)
    {   menunoed *node= &nodes[i]  ;
        if (node->info->parent == MENU_NONE)
        {continue;}
        node->parent = &nodes[node->info->parent];
        if (node->parent->child == NULL){
            node->parent->child = node;
            node->next = node;
            node->last = node;
        }else{
            menunoed *head = node->parent->child;/*fangshuaxing*/
            node->next = head;
            node->last = head->last;
            head->last = node;
            node->last->next = node;
        }
    }
    page = &nodes[MENU_MAIN]; selected = page->child;

    
}

uint8_t menuupdate (KEY_EVENT_ENUM key){
    if ( key == KEY_EVENT_NONE)    {return 0;}
    if ( selected == NULL)
    {
        if (page->info == nodes[MENU_DATA].info){
        if (key == KEY_BACK_SHORT){
        selected = page;
        page = page->parent;
        datashowflag = 0;
                    reflashflag = 1;
        return 1;
        }else{return 0;}    }else{return 0;}

    }
    if ( editing == 1)
    {
        switch ( key)
        {
        case KEY_FORWARD_LONG:
        case KEY_FORWARD_SHORT:
        *(selected->info->value) = editingvalue;
        editing = 0;
            return 1;
        case KEY_UP_LONG:
        case KEY_UP_SHORT:
        editingvalue += selected->info->step;
        if ( editingvalue >= selected->info->max)
        {editingvalue = selected->info->max; }
            return 1;
        case KEY_DOWN_LONG:
        case KEY_DOWN_SHORT:
        editingvalue -= selected->info->step;
        if ( editingvalue <= selected->info->min)
        {editingvalue = selected->info->min; }
            return 1;
        case KEY_BACK_LONG:
        case KEY_BACK_SHORT:
        editing = 0;
             return 1;
        default:
            return 0;
        }
    }
    if ( editing == 0)
    {
        switch (key)
        {
        case KEY_UP_SHORT:                    
        selected = selected->last;
        return 1;

        case KEY_DOWN_SHORT:                    
        selected = selected->next;
        return 1 ;

        case KEY_FORWARD_SHORT:
        if(selected == &nodes[MENU_DATA])
        {   page = selected;
            selected = NULL;
            reflashflag = 1;
        return 1;
        }else if (selected == &nodes[WAVE_SEND_X])
        {
            datachoose(1);
            paramas.xyzchoose = 1;
                    return 1;
        }else if (selected == &nodes[WAVE_SEND_Y])
        {
            datachoose(2);
            paramas.xyzchoose = 2;
                    return 1;
        }else if (selected == &nodes[WAVE_SEND_Z])
        {
            datachoose(3);
            paramas.xyzchoose = 3;
                    return 1;
        }
        else if (selected == &nodes[WAVE_SEND_SAVE])
        {
            shortage();
                    return 1;
        }
        else if (selected == &nodes[DECODER_RESET])
        {
            decoderresetall();   
                    return 1;
        }

        /*else if ( selected == &nodes[MENU_DECODER])
        {page = selected;
         selected = NULL;
        
            break;
        }*/
        else if (selected->child != NULL)
        {
         page =selected;
         selected = selected->child;
            reflashflag = 1;
        return 1;
        }else if (selected->info->value != NULL)
        {
            editing = 1;
            editingvalue = *(selected->info->value);
        return 1;
        }else{
        return 0;
        }

        case KEY_BACK_SHORT:
        if (page->parent != NULL)
        {selected = page;
         page = page->parent;
         decodeshowflag = 0;
            reflashflag = 1;
         return 1;
        }
        default:
        return 0;
        }
    }
    return 0;
}
static MENU_LPF1 lpf[6];
imudata imdt;
void dataget    (void){
    NEUQ_MPU6050_read_data();
     imdt.ax = MPU6050_acc_transition(mpu6050_acc_x);
     imdt.ay = MPU6050_acc_transition(mpu6050_acc_y);
     imdt.az = MPU6050_acc_transition(mpu6050_acc_z);
     imdt.gx = MPU6050_gyro_transition(mpu6050_gyro_x);
     imdt.gy = MPU6050_gyro_transition(mpu6050_gyro_y);
     imdt.gz = MPU6050_gyro_transition(mpu6050_gyro_z);
}/*放在终端里的这个极简函数获取原神数值*/
/*明天写的这个关乎转换的滤波导入函数用于主循环生成新的值，之后是读取，写入fifo上传（主循环），之后是这个save目前就一个a*/
void dataall (void){
float alpha = (100.0f - (float)paramas.flit) / 100.0f;

    for (uint8_t i = 0; i < 6; i++)
    {
        MENU_LPF1_SetAlpha(&lpf[i], alpha);  
    }

    imdt.axpro = MENU_LPF1_Calc(&lpf[0], imdt.ax);
    imdt.aypro = MENU_LPF1_Calc(&lpf[1], imdt.ay);
    imdt.azpro = MENU_LPF1_Calc(&lpf[2], imdt.az);
    imdt.gxpro = MENU_LPF1_Calc(&lpf[3], imdt.gx);
    imdt.gypro = MENU_LPF1_Calc(&lpf[4], imdt.gy);
    imdt.gzpro = MENU_LPF1_Calc(&lpf[5], imdt.gz);
}

void datachoose(int8_t cho){
switch ( cho){
case 1:
    imdt.sdalite = imdt.ax;
    imdt.sdapro = imdt.axpro;
    imdt.sdglite = imdt.gx;
    imdt.sdgpro = imdt.gxpro;
    send();
    break;

case 2:
    imdt.sdalite = imdt.ay;
    imdt.sdapro = imdt.aypro;
    imdt.sdglite = imdt.gy;
    imdt.sdgpro = imdt.gypro;
    send();
    break;

case 3:
    imdt.sdalite = imdt.az;
    imdt.sdapro = imdt.azpro;
    imdt.sdglite = imdt.gz;
    imdt.sdgpro = imdt.gzpro;
    send();
    break;

default:
    break;
}
} 
/*1就是一个选择数据的函数,主循环与update分别调用，标志位启动，包含了后面的发送*/


void send    (void){
    if (paramas.wavestate == 1)
    {
    
    
   if (sendflag == 1 )
   {sendflag = 0;
   }else{return ;}
    float zancun [4];
    uint8_t fasong [20];
    zancun[0] = imdt.sdalite;
    zancun[1] = imdt.sdapro;
    zancun[2] = imdt.sdglite;
    zancun[3] = imdt.sdgpro;
    for (size_t i = 0; i < 4; i++)
    {memcpy(&fasong[i*4],&zancun[i],4);
    }
    fasong[16] = 0x00;      
    fasong[17] = 0x00;
    fasong[18] = 0x80;
    fasong[19] = 0x7F;
    NEUQ_Uart_write_data(UART_0,fasong,20);
}else return ;
}
/*zhuxunhaunxunhaun，放到dtc里*/

void shortage (void){
    NEUQ_Flash_write_sector(511,(uint8_t *)&paramas.flit,4);
}

void mainshow (void){
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
        if (reflashflag
        && page != &nodes[MENU_DATA]
        && page != &nodes[MENU_DECODER])
    {
        NEUQ_LCD200_Clear();
    }
    
    char line[32];
    if (page == NULL){
        return ;
    } 

    if (selected == NULL)
    {if (page == &nodes[MENU_DATA])
    {
        NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_SetPenColor(LCD_BLUE);
        NEUQ_LCD200_Clear();
        datashowflag = 1;
        datashowflag2 = 1;/*qishi没什么卵用，可能跟手一点*/
        datashow();}}
    if (page == &nodes[MENU_DECODER])
    {   NEUQ_LCD200_SetBackColor(LCD_WHITE);
        NEUQ_LCD200_SetPenColor(LCD_BLUE);
        NEUQ_LCD200_Clear();
        decodeshow();
        decodeshowflag = 1;
        decodeshowflag2 = 1;
    }
    sprintf(line,"now page:%s",page->info->zhushi);
    NEUQ_LCD200_ClearRect(10,10,310,24);
    NEUQ_LCD200_DisplayString(10,10,line);
    menunoed *item;
    if (page->child !=NULL)
    {item = page->child;
    int8_t k =1; 
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
        do
    {  if (item == selected )
    {
    NEUQ_LCD200_SetBackColor(LCD_BLUE);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    sprintf (line, "%s",item->info->zhushi);
    NEUQ_LCD200_ClearRect(10,10+k*30,310,24);
    NEUQ_LCD200_DisplayString(10,10+k*30,line);
    if (editing == 1 && item->info->value!=NULL)
    {
        sprintf (line ,"*%d",editingvalue);
//        NEUQ_LCD200_ClearRect(240,10+k*30-30,60,90);
        NEUQ_LCD200_DisplayString(240,10+k*30,line);      
    }else if(item->info->value!=NULL){
        sprintf (line ,">%d",*item->info->value);
//        NEUQ_LCD200_ClearRect(240,10+k*30-30,60,90);
        NEUQ_LCD200_DisplayString(240,10+k*30,line);
    }     
    }else{
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    sprintf (line, "%s",item->info->zhushi);
    NEUQ_LCD200_ClearRect(10,10+k*30,310,24);
    NEUQ_LCD200_DisplayString(10,10+k*30,line);
    }
        k++;
        item = item->next;
    } while (page->child != item);
    
    }
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    
}/*tongyong显示记住加一个他叔判定就是最后一个，*/
void datashow (void){
if (datashowflag && datashowflag2)
{
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    char line2 [32];
    sprintf (line2,"x acc %fg",imdt.ax);
    NEUQ_LCD200_ClearRect(10,40,310,24);
    NEUQ_LCD200_DisplayString(10,40,line2);
    sprintf (line2,"y acc %fg",imdt.ay);
    NEUQ_LCD200_ClearRect(10,70,310,24);
    NEUQ_LCD200_DisplayString(10,70,line2);
    sprintf (line2,"z acc %fg",imdt.az);
    NEUQ_LCD200_ClearRect(10,100,310,24);
    NEUQ_LCD200_DisplayString(10,100,line2);
    sprintf (line2,"x gyro %fdps",imdt.gx);
    NEUQ_LCD200_ClearRect(10,130,310,24);
    NEUQ_LCD200_DisplayString(10,130,line2);
    sprintf (line2,"y gyro %fdps",imdt.gy);
    NEUQ_LCD200_ClearRect(10,160,310,24);
    NEUQ_LCD200_DisplayString(10,160,line2);
    sprintf (line2,"z gyro %fdps",imdt.gz);
    NEUQ_LCD200_ClearRect(10,190,310,24);
    NEUQ_LCD200_DisplayString(10,190,line2);
    datashowflag2 = 0;
}
}/*flag2放在10ms中断，就是当flag1成立时开始10fps的刷新*/
/*关于第一稿界面的实时调用，前面改用标志位，主循环*/

/*明天写task，就是检测刷新与更新，同时更新update的这个return*/
/*接着思源解算以及对应的参数结构体实时读取与显示*/

void menutask (void){
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();

    if(menuupdate (key)){mainshow();}

                reflashflag = 0;
}/*依旧主循环,包含了必要按键解析以及单次显示集合*/

void decodeshow (void){
    
    if (decodeshowflag && decodeshowflag2)
    {
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
        char line3 [32];
        /* [改] decoder 页多了一行重置项（y=70），三行姿态显示整体下移 30px，避免压行 */
        sprintf (line3,"roll  %.1f",e.roll);
        NEUQ_LCD200_ClearRect(10,100,310,24);
        NEUQ_LCD200_DisplayString(10,100,line3);
        sprintf (line3,"pitch %.1f",e.pitch);
        NEUQ_LCD200_ClearRect(10,130,310,24);
        NEUQ_LCD200_DisplayString(10,130,line3);
        sprintf (line3,"yaw   %.1f",e.yaw);
        NEUQ_LCD200_ClearRect(10,160,310,24);
        NEUQ_LCD200_DisplayString(10,160,line3);
        decodeshowflag2 = 0;
    }
}/*上面加的他的特殊显示，同时也要支持标志位的显示，一个用在刚刚进入，另一个用在主循环的盘点*/



#define RAD_TO_DEG 57.29578f
#define DEG_TO_RAD 0.0174533f
#define MAHONY_KP  0.95f    
#define MAHONY_KI  0.05f    

siyuan q = {1.0f,0.0f,0.0f,0.0f};    /* [改] 补 "=" */
float gyro_bias[3] = {0.0f,0.0f,0.0f};   /* [改] 补 ";" */
/* [改] 上电静止标定陀螺零偏：必须在 NEUQ_PIT_init_ms() 之前调用（此时没有中断抢 I2C），
   期间板子必须静止；samples 次 x 2ms，1000 次约 2 秒 */
void decoderreset (int32_t samples){
    float sumgx=0,sumgy=0,sumgz=0;
    if (samples <= 0) {samples = 1000;}   /* [改] 防呆：1000次 x 2ms 约 2 秒 */
    for (int32_t i = 0; i < samples; i++)
    {
        dataget();                       /* [改] 每轮取一组新样本（原来循环里没采样，平均无意义） */
        sumgx += imdt.gx;                /* [改] 零偏统计用原始值 */
        sumgy += imdt.gy;
        sumgz += imdt.gz;                /* [改] 原 sumgy += imdt.gzpro，笔误 */
        NEUQ_Delay_ms(2);                /* [改] 采样间隔 */
    }
    gyro_bias[0] = (sumgx/samples);
    gyro_bias[1] = (sumgy/samples);
    gyro_bias[2] = (sumgz/samples);
}

static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
void mahonyreset (void){
    exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
}

/* [改] 参数：a 单位 g；g 单位 deg/s（与 imdt 一致）；dt 单位 s（10ms 中断 → 0.01f） */
void mahonyupdate (float ax,float ay,float az,float gx,float gy,float gz,float dt){
    float norm = 1 ;
    
    norm = sqrtf (ax*ax+ay*ay+az*az);
    if (norm < 1e-6f) {return;}    /* [改] 零保护：数据无效直接跳过，防止除零产生 NaN */
    ax = ax/norm;
    ay = ay/norm;
    az = az/norm;
    float vx,vy,vz;
    vx = 2.0f * (q.x * q.z - q.w * q.y);
    vy = 2.0f * (q.w * q.x + q.y * q.z);
    vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    float ex,ey,ez;
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* [改] 先扣零偏再换算成 rad/s（原来漏了：零偏算了没用、单位也没换） */
    gx = (gx - gyro_bias[0]) * DEG_TO_RAD;
    gy = (gy - gyro_bias[1]) * DEG_TO_RAD;
    gz = (gz - gyro_bias[2]) * DEG_TO_RAD;

    exInt += ex*dt*MAHONY_KI;
    eyInt += ey*dt*MAHONY_KI;
    ezInt += ez*dt*MAHONY_KI;
    gx = gx + MAHONY_KP*ex +exInt;   /* [改] kp → MAHONY_KP（宏改名） */
    gy = gy + MAHONY_KP*ey +eyInt;
    gz = gz + MAHONY_KP*ez +ezInt;
    float half_dt = 0.5f * dt;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    
    q.w += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    q.x += ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    q.y += ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    q.z += ( q0 * gz + q1 * gy - q2 * gx) * half_dt;
    norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}
sanyuan e;
void Quat_To_Euler(void){
    e.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        e.pitch = (sinp > 0.0f ? 1.0f : -1.0f) * 90.0f; // [改] angles → e；限制在 ±90 度
    } else {
        e.pitch = asinf(sinp) * RAD_TO_DEG;              // [改] angles → e
    }
    e.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;   /* [改] angles → e */

}


void decoderresetall (void){
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    NEUQ_LCD200_ClearRect(10,200,310,24);
    NEUQ_LCD200_DisplayString(10,200,"reset, keep still");

    decoderreset(1000);
    q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    e.roll = 0.0f; e.pitch = 0.0f; e.yaw = 0.0f;
    mahonyreset();
    for (uint8_t i = 0; i < 6; i++){ 
        MENU_LPF1_Init(&lpf[i], (100.0f - (float)paramas.flit) / 100.0f);
    }
}





