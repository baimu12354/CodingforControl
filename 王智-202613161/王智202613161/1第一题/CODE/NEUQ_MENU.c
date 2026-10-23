#include "NEUQ_lib_include.h"






  //遍历存储名称
static const MenuConfig MenuTable[PlanCount]={
    [Plan1]={"Plan1",0},
    [Plan2]={"Plan2",1},
    [Plan3]={"Plan3",2},
    [Plan4]={"Plan4",3},
};




  //定义
static NODE nodes[PlanCount];  //固定菜单内存
static NODE *page;             //当前进入页面
static NODE *select;           //当前选中页面

  //节点链接
static void Menu_Link(void){
    //遍历存入指针,初始化
    for(uint8_t i=0;i<PlanCount;i++){        
        nodes[i].info=&MenuTable[i];
        nodes[i].next=NULL;
        nodes[i].prev=NULL;
    }

    //建立单页（同级）关系
    NODE *first ;
    NODE *last ;
    first = &nodes[0];
    nodes[0].prev=&nodes[0];
    nodes[0].next=&nodes[0];
    
    for(uint8_t i=0;i<PlanCount;i++){
        if(i != 0){
            last=first->prev;
            nodes[i].prev=last;
            last->next=&nodes[i];
            nodes[i].next=first;
            first->prev=&nodes[i];
        }
    }
}

static NODE *pending=NULL;
NODE *working=&nodes[0];
int8_t Pending=0;
int8_t Working=0;
//跳转（点击）逻辑
static uint8_t Menu_Update(KEY_EVENT_ENUM key){
    switch(key){
        case KEY_UP_SHORT:
        case KEY_UP_LONG:
            select=select->prev;
            break;
        case KEY_DOWN_SHORT:
        case KEY_DOWN_LONG:
            select=select->next;
            break;
        case KEY_FORWARD_SHORT:
        case KEY_FORWARD_LONG:
            pending =select;
            Pending=1;
            break;
        case KEY_BACK_LONG:
        case KEY_BACK_SHORT:
            pending =NULL;
            Pending =0;
            break;
        default:
            return 0;  
    }
    return 1;
}

  //显示逻辑
static uint8_t Menu_Show(void){
    NODE *item= &nodes[0];
    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y=8; 
    snprintf(line,sizeof(line),"%s",page->info->name);
    NEUQ_LCD200_DisplayString(8,8,line);

    if(item!=NULL){
        do{ 
            char mark =' ';
            const MenuConfig *INFO =item->info;

            if(item==select){
                mark =':';
                NEUQ_LCD200_SetPenColor(LCD_RED);
                NEUQ_LCD200_ClearRect(8,y,240,24);
            }
            snprintf(line,sizeof(line),"%c %s",mark,INFO->name);    
            NEUQ_LCD200_DisplayString(8,y,line);
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);
            y+=32;
            item=item->next;
        }while(item!=&nodes[0]);
        NEUQ_LCD200_DisplayString(8,y,"Pending :");
        if(Pending==1){
            snprintf(line,sizeof(line),"%s",pending->info->name);
            NEUQ_LCD200_DisplayString(8,y+32,line);}
        NEUQ_LCD200_DisplayString(8,y+64,"Working :");  
            NEUQ_LCD200_ClearRect(8,232,240,24);     
            snprintf(line,sizeof(line),"%s",working->info->name);
            NEUQ_LCD200_DisplayString(8,232,line); 
    
    }
    return 1;
}


void NEUQ_Menu_Init(void){
    Pending=0;
    Working=0;  
    Menu_Link();
    page=&nodes[0];
    select=&nodes[0];
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Parallel);
    NEUQ_LCD200_Clear();
    Menu_Show();
}


void NEUQ_Menu_Mask(void){
    KEY_EVENT_ENUM key;
    key =NEUQ_Key_EventGet();
    if (Menu_Update(key)){
        Menu_Show();
    }
}

PLANS_ID NEUQ_Menu_WorkGet(void){ 
    if((Pending==1)&&(Working==0)){
        working=pending;
        Pending=0;
        pending=NULL;
        Working=1;
        NEUQ_LCD200_ClearRect(8,168,240,24);
    }
    return working->info->planid;
}
