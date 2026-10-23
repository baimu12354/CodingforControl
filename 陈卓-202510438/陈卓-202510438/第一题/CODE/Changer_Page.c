#include "../NEUQ_lib_include.h"
#include "Changer_Page.h"

PAGE_CONTROL PAGE = {0};
uint8_t tag = 0;

void Page_Init(PAGE_CONTROL *PAGE)
{
    PAGE->in[0] = 0;
    PAGE->in[1] = 0;
    PAGE->in[2] = 0;
    PAGE->location = 0;
    PAGE->point = 1;
    PAGE->PWM_WORK = 0;
    PAGE->step = 0.01f;
}

void point_control(STATE *KEY_STATE,PAGE_CONTROL *PAGE)//光标控制
{
    
    if(KEY_STATE->KEY_UP_STATE)
    {
        PAGE->point++;
        KEY_STATE->KEY_UP_STATE = 0;//清除按键标志位
    }
    
    if(KEY_STATE->KEY_DOWN_STATE)
    {
        PAGE->point--;
        KEY_STATE->KEY_DOWN_STATE = 0;//清除按键标志位
    }

    if (KEY_STATE->KEY_IN_STATE)
    {
        PAGE->location ++;
        if (PAGE->location > 4)
        {
            PAGE->location = 4;
        }
        KEY_STATE->KEY_IN_STATE =0;
    }
    
    if (KEY_STATE->KEY_OUT_STATE)
    {   
        if (PAGE->location > 0)
        {
            PAGE->in[PAGE->location-1] = 0;
            PAGE->location --;
            PAGE->point = 1;
            KEY_STATE->KEY_OUT_STATE =0;
        }
        else 
        {
            PAGE->location = 0;
            KEY_STATE->KEY_OUT_STATE =0;
        }
    }
    
}

void MENU(PAGE_CONTROL *PAGE)//主菜单
{
    if(PAGE->in[0] == 0 && PAGE->in[1] == 0)
    {
        if (PAGE->point < 1) PAGE->point =1;
        if (PAGE->point > 2) PAGE->point =2;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 1)/*按确定进入下一页*/
                    {
                        PAGE->in[0] = 1;//page第一位置1，表示进入1-0页，即第一菜单
                        PAGE->point = 1;//光标返回初始位置
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 1)/*按确定进入下一页*/
                    {
                        // NEUQ_MPU6050_init();
                        PAGE->in[0] = 2;//page第一位置2，表示进入1-0页，即第二菜单
                        PAGE->point = 1;//光标返回初始位
                    }
                    break;
        }
    }
}

void LED_PLAN_MENU(PAGE_CONTROL *PAGE)//LED闪烁方案菜单
{
    if(PAGE->in[0] == 1 && PAGE->in[1] == 0)
    {
        if (PAGE->point < 1) PAGE->point =1;
        if (PAGE->point > 4) PAGE->point =4;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        g_next = 0;
                        PAGE->location = 1;
                        tag = 1;
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        g_next = 1;
                        PAGE->location = 1;
                        tag =1;
                    }
                    break;
            case 3:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        g_next = 2;
                        PAGE->location = 1;//光标返回初始位置
                        tag = 1;
                    }
                    break;
            case 4:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        g_next = 3;
                        PAGE->location = 1;//光标返回初始位置
                        tag = 1;
                    }
                    break;
        }
    }
}


void MPU6050_MENU(PAGE_CONTROL *PAGE)//六轴数据处理菜单
{
    if(PAGE->in[0] == 2 && PAGE->in[1] == 0)
    {
        if (PAGE->point < 1) PAGE->point =1;
        if (PAGE->point > 3) PAGE->point =3;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        PAGE->in[1] = 1;//page第二位置1，表示进入2-1页，即第一菜单
                        PAGE->point = 1;//光标返回初始位置
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        PAGE->in[1] = 2;//page第二位置2，表示进入2-2页，即第二菜单
                        PAGE->point = 1;//光标返回初始位
                    }
                    break;
            case 3:
                    /* 光标移动 */
                    if(PAGE->location == 2)/*按确定进入下一页*/
                    {
                        PAGE->in[1] = 3;//page第二位置3，表示进入2-3页，即第三菜单
                        PAGE->point = 1;//光标返回初始位
                    }
                    break;
        }
    }
}

void MPU6050_SETTING_MENU(PAGE_CONTROL *PAGE)//六轴数据处理菜单
{
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2)
    {
        if (PAGE->point < 1) PAGE->point = 1;
        if (PAGE->point > 5) PAGE->point = 5;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 3)/*按确定进入下一页*/
                    {
                        Q2_axis++;
                        if(Q2_axis > 5) Q2_axis = 0;
                        PAGE->location = 2;//光标返回初始位置
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 3)/*按确定进入下一页*/
                    {
                        PAGE->in[2] = 1;
                        PAGE->point = 1;//光标返回初始位置
                    }
                    break;
            case 3:
                    if(PAGE->location == 3)/*按确定进入下一页*/
                    {
                        PAGE->in[2] = 2;
                        PAGE->point = 1;//光标返回初始位置
                    }
                    break;
            case 4:
                    /* 光标移动 */
                    if(PAGE->location == 3)/*按确定进入下一页*/
                    {
                        Q2_send_on ++;
                        if(Q2_send_on > 1) Q2_send_on = 0;
                        PAGE->location = 2;//光标返回初始位

                    }
                    break;
            case 5:
                    /* 光标移动 */
                    if(PAGE->location == 3)/*按确定进入下一页*/
                    {
                        Q2_Config_Save();
                        if(Q2_Config_Save()) NEUQ_LCD200_DisplayString(150,200,"SAVE OK");
                        else NEUQ_LCD200_DisplayString(150,200,"SAVE ERROR");
                        PAGE->location = 2;//光标返回初始位
                        
                    }
                    break;
        }
    }
}

void Q2_Alpha_R_Set(PAGE_CONTROL *PAGE)
{
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 1)
    {
        if (PAGE->point < 1) PAGE->point =1;
        if (PAGE->point > 2) PAGE->point =2;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 4)/*按确定进入下一页*/
                    {
                        Q2_kalman_r += Q2_R_STEP;
                        if(Q2_kalman_r > Q2_R_MAX) Q2_kalman_r = Q2_R_MAX;
                        PAGE->location = 3;//光标返回初始位置
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 4)/*按确定进入下一页*/
                    {
                        Q2_kalman_r -= Q2_R_STEP;
                        if(Q2_kalman_r < Q2_R_MIN) Q2_kalman_r = Q2_R_MIN;
                        PAGE->location = 3;//光标返回初始位
                    }
                    break;
        }
    }
}


void Q2_Alpha_Q_Set(PAGE_CONTROL *PAGE)
{
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 2)
    {
        if (PAGE->point < 1) PAGE->point =1;
        if (PAGE->point > 2) PAGE->point =2;
        switch (PAGE->point)
        {
            case 1:
                    /* 光标移动 */
                    if(PAGE->location == 4)/*按确定进入下一页*/
                    {
                        Q2_kalman_q += Q2_Q_STEP;
                        if(Q2_kalman_q > Q2_Q_MAX) Q2_kalman_q = Q2_Q_MAX;
                        PAGE->location = 3;//光标返回初始位置
                    }
                    break;
            case 2:
                    /* 光标移动 */
                    if(PAGE->location == 4)/*按确定进入下一页*/
                    {
                        Q2_kalman_q -= Q2_Q_STEP;
                        if(Q2_kalman_q < Q2_Q_MIN) Q2_kalman_q = Q2_Q_MIN;
                        PAGE->location = 3;//光标返回初始位
                    }
                    break;
        }
    }
}

void SHOW_SCREEN_WORK(PAGE_CONTROL *PAGE)
{   
    
    SCREEN_CLEAR();
    NEUQ_LCD200_DisplayFloat(10,200,PAGE->location,6,2);
    if (PAGE->in[0] == 0 && PAGE->in[1] == 0) 
    {
        MENU_SHOW();
        POINTER(PAGE->point);
    }
    if(PAGE->in[0] == 1 && PAGE->in[1] == 0)
    {
        LED_PLAN_SHOW();
        LED_WAIT(g_next,g_cur);
        POINTER(PAGE->point);
        
    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 0 && PAGE->in[2] == 0)
    {
        MPU6050_MENU_SHOW();
        POINTER(PAGE->point);
    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 1 && PAGE->in[2] == 0)
    {
        MPU6050_DATA_SHOW();

    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 0)
    {
        Q2_SETTING_SHOW();
        POINTER(PAGE->point);
    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 1)
    {
        ALPHA_SETTING_R_SHOW();
        POINTER(PAGE->point);
    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 2){
        ALPHA_SETTING_Q_SHOW();
        POINTER(PAGE->point);
    }
    if(PAGE->in[0] == 2 && PAGE->in[1] == 3 && PAGE->in[2] == 0)
    {
        Q2_ATTITUDE_SHOW();
        POINTER(PAGE->point);
    }
}

void MENU_SET(PAGE_CONTROL *PAGE)
{
    if (PAGE->in[0] == 0 && PAGE->in[1] == 0) MENU(PAGE);
  
    if (PAGE->in[0] == 1 && PAGE->in[1] == 0) LED_PLAN_MENU(PAGE);
  
    if (PAGE->in[0] == 2 && PAGE->in[1] == 0 && PAGE->in[2] == 0) MPU6050_MENU(PAGE);
    if (PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 0) MPU6050_SETTING_MENU(PAGE);
    if (PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 1) Q2_Alpha_R_Set(PAGE);
    if (PAGE->in[0] == 2 && PAGE->in[1] == 2 && PAGE->in[2] == 2) Q2_Alpha_Q_Set(PAGE);

}