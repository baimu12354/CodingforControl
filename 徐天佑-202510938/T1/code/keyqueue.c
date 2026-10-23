#include "keyqueue.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

uint8_t key_divider = 0;
volatile uint8_t key_pressed[4] = {0, 0, 0, 0};
volatile uint8_t key_last[4] = {0, 0, 0, 0};
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
    uint8_t pressed = (now && !key_last[index]);
    key_last[index] = now;
    return pressed;
}

