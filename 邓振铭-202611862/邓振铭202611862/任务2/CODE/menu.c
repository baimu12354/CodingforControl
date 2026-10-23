#include"NEUQ_lib_include.h"
#include"menu.h"
#include"key.h"
#include"NEUQ_MPU6050.h"
#include"NEUQ_Filter.h"
MenuPage_enum MenuPage_now = 0;
uint16_t selected = 0;
uint16_t selected_filter=0;
uint8_t sel_axis=0;
uint16_t alpha_eidt_mode=0;
typedef struct
{
    /* data */
const char*name;
menuid parent_id;
int16_t max;
int16_t min;
int16_t step;
int16_t *value;

}menustruct;
#define PAGE_CNT 4
 const menutable_t menutable[menu_count]={
{menu_none,"None",NULL},
{menu_main,"main",draw_main},
{menu_datashow,"liu zhou zi tai jie suan ",draw_data},
{menu_lvbo_set,"bo de she zhi",draw_filter},
{menu_jitaijiesuan,"zi tai jie suan",draw_attitude}

};

// 固定宽度浮点显示：配合"不清屏刷新"避免残留与闪屏
static void show_val(uint16_t x, uint16_t y, float v){
    char buf[16];
    snprintf(buf, sizeof(buf), "%6.2f", v);
    NEUQ_LCD200_DisplayString(x, y, buf);
}

static void draw_main(void){
NEUQ_LCD200_Clear();
NEUQ_LCD200_DisplayString(0,0,"MAIN menu");
if(selected==0){
    NEUQ_LCD200_DisplayString(0,40,">liu zhou zi tai jie suan");

}else{
    NEUQ_LCD200_DisplayString(0,40,"liu zhou zi tai jie suan");
}
if(selected==1){
    NEUQ_LCD200_DisplayString(0,80,">bo de she zhi");
}else{
NEUQ_LCD200_DisplayString(0,80,"bo de she zhi");
}
if(selected==2){
    NEUQ_LCD200_DisplayString(0,120,">zi tai jie suan");

}
else{
    NEUQ_LCD200_DisplayString(0,120,"zi tai jie suan");
}
}
static void draw_data(void){
    NEUQ_LCD200_DisplayString(0,0,"MPU6050DATA");
    float ax=MPU6050_acc_transition(mpu6050_acc_x);
    float ay=MPU6050_acc_transition(mpu6050_acc_y);
    float az=MPU6050_acc_transition(mpu6050_acc_z);
    float gx=mpu6050_gyro_x/16.4f;
    float gy=mpu6050_gyro_y/16.4f;
    float gz=mpu6050_gyro_z/16.4f;
    NEUQ_LCD200_DisplayString(0,20,"ax:");
    show_val(40,20,ax);
    NEUQ_LCD200_DisplayString(160,20,"g");
    NEUQ_LCD200_DisplayString(0,50,"ay:");
    show_val(40,50,ay);
    NEUQ_LCD200_DisplayString(160,50,"g");
    NEUQ_LCD200_DisplayString(0,80,"az:");
    show_val(40,80,az);
    NEUQ_LCD200_DisplayString(160,80,"g");
    NEUQ_LCD200_DisplayString(0,110,"gx:");
    show_val(40,110,gx);
    NEUQ_LCD200_DisplayString(160,110,"deg/s");
    NEUQ_LCD200_DisplayString(0,140,"gy:");
    show_val(40,140,gy);
    NEUQ_LCD200_DisplayString(160,140,"deg/s");
    NEUQ_LCD200_DisplayString(0,170,"gz:");
    show_val(40,170,gz);
    NEUQ_LCD200_DisplayString(160,170,"deg/s");
}
static void draw_filter(void){
NEUQ_LCD200_Clear();
NEUQ_LCD200_DisplayString(0,0,"Filter Set Page");
NEUQ_LCD200_DisplayString(0,30,"Axis:");
float show_alpha;
switch (sel_axis)
{
case 0:
NEUQ_LCD200_DisplayString(70,30,"ax");
show_alpha=filter_ax.alpha;
break;
case 1:
NEUQ_LCD200_DisplayString(70,30,"ay");
show_alpha=filter_ay.alpha;
break;
case 2:
NEUQ_LCD200_DisplayString(70,30,"az");
show_alpha=filter_az.alpha;
break;
case 3:
NEUQ_LCD200_DisplayString(70,30,"gx");
show_alpha=filter_gx.alpha;
break;
case 4:
NEUQ_LCD200_DisplayString(70,30,"gy");
show_alpha=filter_gy.alpha;
break;
case 5:
NEUQ_LCD200_DisplayString(70,30,"gz");
show_alpha=filter_gz.alpha;
break;
}
NEUQ_LCD200_DisplayString(0,60,"alpha:");
NEUQ_LCD200_DisplayFloat(120,60,show_alpha,6,2);
NEUQ_LCD200_DisplayString(0,90,"SendVOFA:");
if(vofa_en){
    NEUQ_LCD200_DisplayString(120,90,"ON");

}
else{
    NEUQ_LCD200_DisplayString(120,90,"OFF");
}
NEUQ_LCD200_DisplayString(0,120,"save to flash");

NEUQ_LCD200_DisplayString(200,30+selected_filter*30,"<----");
}

extern float roll;
extern float pitch;
extern float yaw;
//等等在写
static void draw_attitude(void){
NEUQ_LCD200_DisplayString(0,0,"Attitude Solve");
NEUQ_LCD200_DisplayString(0,30,"Roll:");
show_val(60,30,roll);
NEUQ_LCD200_DisplayString(120,30,"deg");
NEUQ_LCD200_DisplayString(0,60,"pitch");
show_val(60,60,pitch);
NEUQ_LCD200_DisplayString(120,60,"deg");
NEUQ_LCD200_DisplayString(0,90,"Yaw:");
show_val(60,90,yaw);
NEUQ_LCD200_DisplayString(120,90,"deg");
}




void menu_init(void){
MenuPage_now=page_main;
selected=0;
sel_axis=0;
selected_filter=0;
alpha_eidt_mode=0;
NEUQ_LCD200_SetPenColor(LCD_BLACK);
NEUQ_LCD200_SetBackColor(LCD_WHITE);

}

void menu_task(void){
    uint16_t key=key_eventget();
    //if(key!=0xFF) printf("KEY:%d\r\n", key);   // 【临时调试】按下按键打印编号，定位退出键，测完删掉
    uint8_t redraw=0;
   
    if(MenuPage_now==page_main){
        if(key==0){selected=(selected==0)?2:selected-1; redraw=1;}
        else if(key==1){selected=(selected==2)?0:selected+1; redraw=1;}
        else if(key==2){
            if(selected==0)      MenuPage_now=page_datashow;
            else if(selected==1){MenuPage_now=page_lvbo_set; sel_axis=0; selected_filter=0; alpha_eidt_mode=0;}
            else                 MenuPage_now=page_jitaijiesuan;
        }
    }
    else if(MenuPage_now==page_datashow){
        if(key==3) MenuPage_now=page_main;  
    }
    else if(MenuPage_now==page_jitaijiesuan){
        if(key==3) MenuPage_now=page_main;
    }
    else if(MenuPage_now==page_lvbo_set){
        if(alpha_eidt_mode==0){
            // 光标模式
            if(key==3)      MenuPage_now=page_main;
            else if(key==0){selected_filter=(selected_filter==0)?3:selected_filter-1; redraw=1;}
            else if(key==1){selected_filter=(selected_filter==3)?0:selected_filter+1; redraw=1;}
            else if(key==2){
                if(selected_filter<=1)      alpha_eidt_mode=1;   // Axis / alpha 进入编辑
                else if(selected_filter==2){vofa_en=!vofa_en; redraw=1;}
                else                        Filter_SaveToFlash();
            }
        }else{
            // 编辑模式selected_filter
            if(key==3) alpha_eidt_mode=0;
            else if(selected_filter==0){
                if(key==0){sel_axis=(sel_axis==0)?5:sel_axis-1; redraw=1;}
                else if(key==1){sel_axis=(sel_axis==5)?0:sel_axis+1; redraw=1;}
            }
            else if(selected_filter==1){
                float *p_alpha=NULL;
                switch(sel_axis){
                    case 0:p_alpha=&filter_ax.alpha;break;
                    case 1:p_alpha=&filter_ay.alpha;break;
                    case 2:p_alpha=&filter_az.alpha;break;
                    case 3:p_alpha=&filter_gx.alpha;break;
                    case 4:p_alpha=&filter_gy.alpha;break;
                    case 5:p_alpha=&filter_gz.alpha;break;
                }
                if(p_alpha){
                    if(key==0)      *p_alpha+=0.01f;
                    else if(key==1) *p_alpha-=0.01f;
                    if(*p_alpha>1.0f)  *p_alpha=1.0f;
                    if(*p_alpha<0.01f) *p_alpha=0.01f;
                    redraw=1;
                }
            }
        }
    }

    // 绘制（避免闪屏：只在切页时清屏一次；数据/姿态页每帧只覆盖刷新数值，不清屏）
    static MenuPage_enum last_page=(MenuPage_enum)0xFF;
    if(last_page != MenuPage_now){
        NEUQ_LCD200_Clear();
    }
    if(MenuPage_now==page_datashow){
        draw_data();
    }
    else if(MenuPage_now==page_jitaijiesuan){
        draw_attitude();
    }
    else if(redraw || last_page!=MenuPage_now){
        if(MenuPage_now==page_main)      draw_main();
        else if(MenuPage_now==page_lvbo_set) draw_filter();
    }
    last_page=MenuPage_now;
}
