
#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stdbool.h>



    
typedef struct {
    uint8_t head;
    uint8_t tail;
} key_queue_t;
extern  key_queue_t g_key_queue1 ;
extern  key_queue_t g_key_queue2 ;
extern  key_queue_t g_key_queue3 ;
extern  key_queue_t g_key_queue4 ;
 extern uint8_t key_divider ;
bool key_queue_push(uint8_t key_id);
bool key_queue_pop(uint8_t keyid);

#endif