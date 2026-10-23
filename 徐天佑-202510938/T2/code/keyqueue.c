#include "keyqueue.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "menu.h"
#include "flash.h"
#include "lcd.h"
volatile uint8_t key_last[4] = {0, 0, 0, 0};
uint8_t key_divider = 0;
uint8_t editing=0;
key_queue_t g_key_queue1 = {0};
key_queue_t g_key_queue2 = {0};
key_queue_t g_key_queue3 = {0};
key_queue_t g_key_queue4 = {0};



bool key_queue_push(uint8_t key_id)
{
    key_queue_t *g_key_queue;
    switch (key_id) {
        case 0: g_key_queue = &g_key_queue1; break;
        case 1: g_key_queue = &g_key_queue2; break;
        case 2: g_key_queue = &g_key_queue3; break;
        case 3: g_key_queue = &g_key_queue4; break;
        default: return false;
    }
    uint8_t next_head = (g_key_queue->head + 1) ;
    if(next_head > 8) {
        next_head = 0;
    }
    

    if (next_head == g_key_queue->tail) {
        return false;
    }
       
    g_key_queue->head = next_head;
    return true;
}


bool key_queue_pop(uint8_t keyid)
{
    key_queue_t* g_key_queue;
    switch (keyid) 
    {
        case 0: g_key_queue = &g_key_queue1; break;
        case 1: g_key_queue = &g_key_queue2; break;
        case 2: g_key_queue = &g_key_queue3; break;
        case 3: g_key_queue = &g_key_queue4; break;
        default: return false;
    }
    if (g_key_queue->head == g_key_queue->tail) 
    {
        return false;
    }
    g_key_queue->tail = (g_key_queue->tail + 1);
    if(g_key_queue->tail > 8) {
        g_key_queue->tail = 0;
    }
    return true;
}



uint8_t key_read(GPIO_Ports_ENUM key, uint8_t index)
{
    uint8_t now = (NEUQ_GPIO_read(key) == GPIO_Low) ? 1 : 0;
    uint8_t pressed = (now && !key_last[index]) ? 1 : 0;
    key_last[index] = now;
    return pressed;
}


void KeyProcess(key_name key)
{
    uint8_t id;
    if (!editing) {
        if (key == KEY_EVENTDOWN) 
        {
            lastselected = selected;
            selected = selected->next;

        }
        else if (key == KEY_EVENTUP) 
        {
            lastselected = selected;
            selected = selected->prev;
        }
        else if (key == KEY_EVENTBACK && page->parent != NULL) 
        { 
            selected = page; 
            page = page->parent; 
            clear=1;
        }
        else if (key == KEY_EVENTENTER) 
        {
            if (selected->child != NULL) 
            { 
                page = selected; 
                selected = page->child;
                clear=1;
            }
            else if (selected->info->editable) 
            { 
                editing = 1; 
                edit_value = *selected->info->value; 

            }
            else 
            {
                id = MenuNodeId(selected);
                 if (id >= MENU_AX && id <= MENU_GZ ) 
                 {

                         vofa_axis = (AXIS)(id - MENU_AX);
                        *selected->info->value = 1.0f - *selected->info->value;
                            musave = 1;
                     
                 }
 
                
            }


        }
        return;
    }
    if (key == KEY_EVENTUP) edit_value += 0.05f ;
    else if (key == KEY_EVENTDOWN) edit_value -=0.05f;
    else if (key == KEY_EVENTENTER) 
    {

   
        if (MenuNodeId(selected) == MENU_ALPHA_ACC || MenuNodeId(selected) == MENU_ALPHA_GYRO)
        {
            if (edit_value < TASK2_FILTER_MIN) edit_value = TASK2_FILTER_MIN;
            if (edit_value > TASK2_FILTER_MAX) edit_value = TASK2_FILTER_MAX; 
            
            *selected->info->value = edit_value;
        }
        if (MenuNodeId(selected) == MENU_VOFA_ENABLE) 
        {
        *selected->info->value = (1-*selected->info->value);
        }
        musave = 1;
        editing = 0;
    } else if (key == KEY_EVENTBACK)  editing = 0;


}


