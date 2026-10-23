#include "NEUQ_lib_include.h"

NEUQ_Menu_Params params; 




//页面编号
typedef enum{
    MENU_NONE=-1,
    MAIN=0,
    mode1,
    mode2,
    mode3,
    child1,
    child3,
    filter_param,     // 滤波参数
    upload_axis,      // 发送轴
    upload_on,        // 上传开关
    MENU_COUNT,
}MenuID;


//页面信息配
typedef struct {
    const char *name;    //当前页面名称 
    MenuID parent_id;    //当前页面父节点编号
    int32_t *value;      //当前页面参数地址
    int32_t min;
    int32_t max;
    int32_t step;
}MenuConfig;

static const MenuConfig MenuTable[MENU_COUNT]={
    [MAIN]={"Main menu",MENU_NONE,NULL,0,0,0},
    [mode1]={"6-Axis Data",MAIN,NULL,0,0,0},
    [mode2]={"Filter Set",MAIN,NULL,0,0,0},
    [mode3]={"Attitude Calc",MAIN,NULL,0,0,0},
    [child1]={"Child 1",mode1,NULL,0,0,0},
    [child3]={"Child 3",mode3,NULL,0,0,0},
    [filter_param]={"Filter Param",mode2,&params.FilterParams,1,100,1},
    [upload_axis] ={"Upload Axis",mode2,&params.UploadAxis,0,6,1},
    [upload_on]   ={"Upload On/Off",mode2,&params.UploadOn,0,1,1},
};
   
//链表节点结构
typedef struct MenuNode{
    const MenuConfig *info;  //节点内容
    struct MenuNode *parent; //父节点
    struct MenuNode *child;  //子节点
    struct MenuNode *prev;   //同级前节点
    struct MenuNode *next;  //同级后节点
}MenuNode;

static MenuNode nodes[MENU_COUNT];//固定内存存储节点
static MenuNode *page;            //当前进入页面
static MenuNode *selected;        //当前选项（selected 永远是page 的子节点）
static uint8_t  editing;          //0：浏览 1：编辑

    //根据菜单表格去构建节点链接关系
static void Menu_BuildLink(void){
    for (uint8_t i=0;i<MENU_COUNT;i++){
        nodes[i].info=&MenuTable[i];
        nodes[i].parent=NULL;
        nodes[i].child=NULL;
        nodes[i].next=NULL;
        nodes[i].prev=NULL;
    }
    //第二步，构建层级关系和同级关系
    for (uint8_t i=0;i<MENU_COUNT;i++){
        
        //层级关系
        MenuNode *node=&nodes[i];
        MenuNode *Parent;
        if(node->info->parent_id ==MENU_NONE) continue;
        Parent=&nodes[node->info->parent_id]; 
        node->parent=Parent;
        
        //同级关系
        MenuNode *first;
        MenuNode *last;
        first =Parent->child;
        if(first==NULL){
            Parent->child=node;
            node->prev=node;
            node->next=node;
        }else{
            last=first->prev;
            node->prev=last;
            last->next=node;
            node->next=first;
            first->prev=node;
        }
    }
}

//选轴辅助显示
static void NEUQ_AxisShow(int32_t value){
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_SetBackColor(LCD_BLUE);
    switch (value)
    {
    case 1:
        NEUQ_LCD200_DisplayString(200,80,"Ax");
        break;
    case 2:
        NEUQ_LCD200_DisplayString(200,80,"Ay");
        break;
    case 3:
        NEUQ_LCD200_DisplayString(200,80,"Az");
        break;
    case 4:
        NEUQ_LCD200_DisplayString(200,80,"Gx");
        break;
    case 5:
        NEUQ_LCD200_DisplayString(200,80,"Gy");
        break;
    case 6:
        NEUQ_LCD200_DisplayString(200,80,"Gz");
        break;
    default:
        break;
    }
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
}


int32_t edit_value ;
//跳转逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    //编辑模式
    if(editing==1){
        const MenuConfig *item =selected->info;

        switch (key){
            case KEY_UP_SHORT:           
                edit_value += item->step; 
                Menu_Show();
                if(item ->value ==&params.UploadAxis) {
                    NEUQ_AxisShow(edit_value);
                }        
                break;
            case KEY_DOEN_SHORT:           
                edit_value -= item->step;
                Menu_Show(); 
                if(item ->value ==&params.UploadAxis) {
                    NEUQ_AxisShow(edit_value);
                }  
                break;
            case KEY_FORWARD_SHORT:           
                *item->value=edit_value;
                if(selected == &nodes[filter_param]){
                NEUQ_MPU_SaveParam();
            }
                editing=0;
                Menu_Show();
                if(item ->value ==&params.UploadAxis) {
                    NEUQ_AxisShow(edit_value);
                }   
                break;
            case KEY_BACK_SHORT:           
                editing=0;  
                Menu_Show(); 
                if(item ->value ==&params.UploadAxis) {
                    NEUQ_AxisShow(edit_value);
                }    
                break;
            default:
                return 0;
        
        }
        if(edit_value < item->min) edit_value =item->min;
        if(edit_value > item->max) edit_value =item->max;
        return 0;
    }

    //浏览模式
    switch (key){
        case KEY_UP_SHORT:
            selected=selected->prev;
            break;
        case KEY_DOEN_SHORT:
            selected=selected->next;
            break;
        case KEY_FORWARD_SHORT:
            if(selected->child !=NULL){
                page =selected;
                selected=selected->child;
            }else if(selected->info->value !=NULL){
                editing=1;
                edit_value=*selected->info->value;
            }else{
                return 0;
            }
            break;
        case KEY_BACK_SHORT:
            if( page ->parent==NULL) return 0;
            selected=page;
            page=page->parent;
            break;
        default:
            return 0;
    }
    return 1;
}

static void Mode1_Show(void){
       char line[32];
       uint16_t y=48;
       
       NEUQ_LCD200_DisplayString(16,y,"Ax");
       NEUQ_LCD200_ClearRect(56,y,120,24);
       NEUQ_LCD200_DisplayFloat(56,y,MPU6050_acc_transition(mpu6050_acc_x),6,2);
       NEUQ_LCD200_DisplayString(120,y,"g");
       NEUQ_LCD200_DisplayString(140,y,"Gx");
       NEUQ_LCD200_ClearRect(176,y,64,24);
       NEUQ_LCD200_DisplayFloat(176,y,NEUQ_MPU_GetCalibratedGyro(4),6,2);
       NEUQ_LCD200_DisplayString(240,y,"deg/s");
       y+=32;
       NEUQ_LCD200_DisplayString(16,y,"Ay");
       NEUQ_LCD200_ClearRect(56,y,120,24);
       NEUQ_LCD200_DisplayFloat(56,y,MPU6050_acc_transition(mpu6050_acc_y),6,2);
       NEUQ_LCD200_DisplayString(120,y,"g");
       NEUQ_LCD200_DisplayString(140,y,"Gy");
       NEUQ_LCD200_ClearRect(176,y,64,24);
       NEUQ_LCD200_DisplayFloat(176,y,NEUQ_MPU_GetCalibratedGyro(5),6,2);
       NEUQ_LCD200_DisplayString(240,y,"deg/s");
       y+=32;
       NEUQ_LCD200_DisplayString(16,y,"Az");
       NEUQ_LCD200_ClearRect(56,y,120,24);
       NEUQ_LCD200_DisplayFloat(56,y,MPU6050_acc_transition(mpu6050_acc_z),6,2);
       NEUQ_LCD200_DisplayString(120,y,"g");
       NEUQ_LCD200_DisplayString(140,y,"Gz");
       NEUQ_LCD200_ClearRect(176,y,64,24);
       NEUQ_LCD200_DisplayFloat(176,y,NEUQ_MPU_GetCalibratedGyro(6),6,2);
       NEUQ_LCD200_DisplayString(240,y,"deg/s");
       y+=32;
};
//显示逻辑
static int Menu_Show(void){
    MenuNode *item=page->child;
    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y=48; 
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if(page==&nodes[mode1]){
        Mode1_Show();
    }else if(page==&nodes[mode3]){
        NEUQ_MPU_ShowAttitude();
    }else if(item!=NULL){

        do{
            char mark =' ';
            const MenuConfig *INFO =item->info;

            if(item==selected){
                mark =editing?'*':':';
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
            NEUQ_LCD200_ClearRect(8,y,320,24);
            }

            if(INFO->value != NULL){
                int32_t value =((item==selected)&&(editing))?edit_value : (*INFO->value);
                snprintf(line,sizeof(line),"%c %s:%ld",mark,INFO->name,value);
            }else{
                snprintf(line,sizeof(line),"%c %s",mark,INFO->name);    
            }
            NEUQ_LCD200_DisplayString(8,y,line);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item=item->next;

        }while(item!=page->child);
    }
    return 1;
}

void NEUQ_Menu_Init(void){
    params.UploadAxis=0;
    params.UploadOn=0;
    params.FilterParams=0;
    Menu_BuildLink();
    page=&nodes[MAIN];
    selected= page->child;
    editing =0;

    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
    Menu_Show();
}
extern volatile uint32_t cnt_ms;
void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key;
    static uint32_t last_refresh = 0;
    key =NEUQ_Key_EventGet();
    if (Menu_Update(key)){
        Menu_Show();
        last_refresh = cnt_ms;
    }
    if ((page == &nodes[mode1]) &&(uint32_t)(cnt_ms - last_refresh) >= 100u){   // 100ms 刷新一次
        last_refresh = cnt_ms;
        Mode1_Show();
    }
    if ((page == &nodes[mode3]) &&(uint32_t)(cnt_ms - last_refresh) >= 100u){   // 100ms 刷新一次
        last_refresh = cnt_ms;
        NEUQ_MPU_ShowAttitude();
    }
}