#include "NEUQ_lib_include.h"

volatile NULLBIAS_STR NB_PREV={0};

// Menu_Params params;

uint8_t filterShow=0;

typedef enum
{
    MENU_NULL=-1,MENU_MAIN=0,
    MENU_GYRODATA=1,MENU_FILTER,MENU_ANGLE,
    MENU_AX,MENU_AY,MENU_AZ,MENU_GX,MENU_GY,MENU_GZ,
    MENU_F_AX,MENU_F_AY,MENU_F_AZ,MENU_F_GX,MENU_F_GY,MENU_F_GZ,
    MENU_BIASPREVE,MENU_ANGLE_ROLL,MENU_ANGLE_PITCH,MENU_ANGLE_YAW,
    MENU_NUM
}MenuID;

typedef struct 
{
    const char* name;
    MenuID myID;
    MenuID parentID;
    volatile int16_t *valueCheck;//这个const用于确保数据不被修改，是否正确需要验证
    volatile uint8_t *value;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    /* data */
}MenuConfig;




// filters[FLITER_AX];

static const MenuConfig MenuTable[MENU_NUM]=
{
    [MENU_MAIN]={"Main",MENU_MAIN,MENU_NULL,NULL,NULL,0,0,0},
    [MENU_GYRODATA]={"GYRODATA",MENU_GYRODATA,MENU_MAIN,NULL,NULL,0,0,0},
    [MENU_FILTER]={"FILTER",MENU_FILTER,MENU_MAIN,NULL,NULL,0,0,0},
    [MENU_ANGLE]={"ANGLE",MENU_ANGLE,MENU_MAIN,NULL,NULL,0,0,0},
    [MENU_AX]={"ax_g",MENU_AX,MENU_GYRODATA,&filters[FILTER_AX].final,NULL,0,0,0},
    [MENU_AY]={"ay_g",MENU_AY,MENU_GYRODATA,&filters[FILTER_AY].final,NULL,0,0,0},
    [MENU_AZ]={"az_g",MENU_AZ,MENU_GYRODATA,&filters[FILTER_AZ].final,NULL,0,0,0},
    [MENU_GX]={"gx_deg/s",MENU_GX,MENU_GYRODATA,&filters[FILTER_GX].final,NULL,0,0,0},
    [MENU_GY]={"gy_deg/s",MENU_GY,MENU_GYRODATA,&filters[FILTER_GY].final,NULL,0,0,0},
    [MENU_GZ]={"gz_deg/s",MENU_GY,MENU_GYRODATA,&filters[FILTER_GZ].final,NULL,0,0,0},
    [MENU_F_AX]={"F_AX",MENU_F_AX,MENU_FILTER,NULL,&filters[FILTER_AX].F_value,0,10,1},
    [MENU_F_AY]={"F_AY",MENU_F_AY,MENU_FILTER,NULL,&filters[FILTER_AY].F_value,0,10,1},
    [MENU_F_AZ]={"F_AZ",MENU_F_AZ,MENU_FILTER,NULL,&filters[FILTER_AZ].F_value,0,10,1},
    [MENU_F_GX]={"F_GX",MENU_F_GX,MENU_FILTER,NULL,&filters[FILTER_GX].F_value,0,10,1},
    [MENU_F_GY]={"F_GY",MENU_F_GY,MENU_FILTER,NULL,&filters[FILTER_GY].F_value,0,10,1},
    [MENU_F_GZ]={"F_GZ",MENU_F_GZ,MENU_FILTER,NULL,&filters[FILTER_GZ].F_value,0,10,1},
    [MENU_BIASPREVE]={"BIASPREVE",MENU_BIASPREVE,MENU_ANGLE,NULL,&NB_PREV.editing_NullBias_Flag,0,1,1},
    [MENU_ANGLE_ROLL]={"ROLL",MENU_ANGLE_ROLL,MENU_ANGLE,NULL,NULL,0,0,0},
    [MENU_ANGLE_PITCH]={"PITCH",MENU_ANGLE_PITCH,MENU_ANGLE,NULL,NULL,0,0,0},
    [MENU_ANGLE_YAW]={"YAW",MENU_ANGLE_YAW,MENU_ANGLE,NULL,NULL,0,0,0}
};

typedef struct MenuNode
{
    const MenuConfig* infor;
    struct MenuNode* parent;
    struct MenuNode* child;
    struct MenuNode* prev;
    struct MenuNode* next;
}MenuNode;

static MenuNode nodes[MENU_NUM];
static MenuNode* page;
static MenuNode* selected;
static uint8_t editing;

static void menuBuild(void)
{
    for(uint8_t i=0;i<MENU_NUM;i++)
    {
        nodes[i].infor=&MenuTable[i];
        nodes[i].parent=nodes[i].child=nodes[i].prev=nodes[i].next=NULL;
    }

    for(uint8_t i=0;i<MENU_NUM;i++)
    {
        if(nodes[i].infor->parentID==MENU_NULL)continue;
        nodes[i].parent=&nodes[nodes[i].infor->parentID];
        MenuNode* first;
        MenuNode* last;
        first=nodes[i].parent->child;
        if(first==NULL)
        {
            nodes[i].parent->child=&nodes[i];
            nodes[i].next=&nodes[i];
            nodes[i].prev=&nodes[i];
        }
        else
        {
            last=first->prev;
            first->prev=&nodes[i];
            last->next=&nodes[i];
            nodes[i].next=first;
            nodes[i].prev=last;
        }
    }

}


uint8_t edit_value;


static uint8_t Menu_Update(KEY_EVENT_ENUM key)
{
    if(editing==1)
    {
        const MenuConfig* item=selected->infor;
        switch (key)
        {
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            edit_value+=item->step;
            break;

        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            edit_value-=item->step;
            break;
        
        case KEY_FORWARD_SHORT:
            *item->value=edit_value;
            editing=0;
            
            for(uint8_t i=0;i<6;i++)
            {
                dataWrite[i]=filters[FILTER_AX+i].F_value;
            }
            NEUQ_Flash_write_sector(FLASH_DATA_STORE
                ,(const uint8_t*)dataWrite,(uint16_t)sizeof(dataWrite));
        // NEUQ_Flash_page_program(NEUQ_Flash_get_sector_address(FLASH_DATA_STORE)
        // ,(const uint8_t*)dataWrite,(uint16_t)sizeof(dataWrite));

            break;

        case KEY_BACK_SHORT:
            editing=0;
            break;

        default:
        return 0;

        }
        if(edit_value>item->max)edit_value=item->max;
        if(edit_value<item->min)edit_value=item->min;
        return 1;
    }

    switch (key)
    {
    case KEY_UP_SHORT:
        selected=selected->prev;
        break;

    case KEY_DOWN_SHORT:
        selected=selected->next;
        break;
    
    case KEY_FORWARD_SHORT:
        if (selected->child!=NULL)
        {
            page=selected;
            selected=selected->child;
        }
        else if (selected->infor->value!=NULL)
        {
            editing=1;
            edit_value=*selected->infor->value;
        }
        // else if(selected->infor->valueCheck!=NULL)
        // {
        //     check=1;
        // }
        else 
        {
            return 0;
        }
        break;
    case KEY_BACK_SHORT:
        if(page->parent!=NULL)
        {
            selected=page;
            page=page->parent;
        }
        break;
    default:
        return 0;
    }
    return 1;
}


void Menu_Show(void)
{
    MenuNode* item=page->child;
    char line[32];
    uint16_t y=48;
    NEUQ_LCD200_Clear();
    snprintf(line,sizeof(line),"%s",page->infor->name);
    NEUQ_LCD200_DisplayString(8,8,line);
    NEUQ_MPU6050_read_data();
    if(item != NULL)
    {
        do{
            char mark=' ';
            const MenuConfig* info=item->infor;

            if(item == selected)
            {
                mark=editing?'*':'>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8,y,320,24);
            }
            if(info->value!=NULL)
            {
                int32_t value=((item==selected)&&(editing))?edit_value:(*info->value);
                snprintf(line,sizeof(line),"%c %s:%ld",mark,info->name,value);
            }
            // else if(info->valueCheck!=NULL)//need to test
            // {
            //     snprintf(line,sizeof(line),"%c %s:%.2f",
            //     mark,info->name,(MPU6050_acc_transition(*(info->valueCheck))));
            //  }
            else
            {
                snprintf(line,sizeof(line),"%c %s",mark,info->name);
            }

            NEUQ_LCD200_DisplayString(8,y,line);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);

            y+=32;
            item=item->next;
        } while(item!=page->child);
    }

}


void Menu_Init(void)
{
    menuBuild();
    page=&nodes[MENU_MAIN];
    selected=page->child;
    editing=0;
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_Clear();
    Menu_Show();
}


void Menu_ShowAdd(void)
{
    char show[24];
    if(page==&nodes[MENU_GYRODATA])/*显示数据*/
    {
        // NEUQ_LCD200_Display_int32(40,100,1,10);//TEST
        // NEUQ_MPU6050_read_data();


        for(uint8_t i=0;i<3;i++)//由于需要频繁刷新没法写到menu——show
        {
            snprintf(show,sizeof(show),"%.2f",(MPU6050_acc_transition(*nodes[MENU_AX+i].infor->valueCheck)));
            NEUQ_LCD200_DisplayString(150,48+i*32,show);
        }
        for(uint8_t i=0;i<3;i++)
        {
            snprintf(show,sizeof(show),"%.2f",(MPU6050_gyro_transition(*nodes[MENU_GX+i].infor->valueCheck)));
            NEUQ_LCD200_DisplayString(150,48+(i+3)*32,show);
        }
    }
    if(page==&nodes[MENU_FILTER]&&editing==0)/*滤波显示*/
    {
        switch (selected->infor->myID)
        {
        case MENU_F_AX:
        case MENU_F_AY:
        case MENU_F_AZ:
        printf("%.2f,%.2f\n",MPU6050_acc_transition(filters[selected->infor->myID-MENU_F_AX].new),//用减法把数字“归零”，滤波ax是0
        MPU6050_acc_transition(filters[selected->infor->myID-10].final));//和上位机通信
            break;
        case MENU_F_GX:
        case MENU_F_GY:
        case MENU_F_GZ:
        printf("%.2f,%.2f\n",MPU6050_gyro_transition(filters[selected->infor->myID-MENU_F_AX].new),
        MPU6050_gyro_transition(filters[selected->infor->myID-10].final));
            break;
        default:
            break;
        }
    }
    if(page==&nodes[MENU_ANGLE])
    {
        
        snprintf(show,sizeof(show),"%.2f",Calcu_Value.roll);
        NEUQ_LCD200_DisplayString(150,48+1*32,show);
        snprintf(show,sizeof(show),"%.2f",Calcu_Value.pitch);
        NEUQ_LCD200_DisplayString(150,48+2*32,show);
        snprintf(show,sizeof(show),"%.2f",Calcu_Value.yaw);
        NEUQ_LCD200_DisplayString(150,48+3*32,show);
        
        // NEUQ_LCD200_Display_int32(10,180,NB_PREV.editing_NullBias_Flag,2);//test
        // snprintf(show,sizeof(show),"%.2f",NB_PREV.sum_gx);
        // NEUQ_LCD200_DisplayString(150,48+4*32,show);
        // snprintf(show,sizeof(show),"%.2f",NB_PREV.gx_bias);
        // NEUQ_LCD200_DisplayString(150,48+5*32,show);
    }
    
}

void Menu_Task(void)
{
    KEY_EVENT_ENUM key;

    key=key_eventGet();
    if(Menu_Update(key))
    {
        Menu_Show();
    }
    Menu_ShowAdd();
    
}

