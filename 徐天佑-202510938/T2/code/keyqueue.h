
#ifndef QUEUE_H
#define QUEUE_H
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include <stdint.h>
#include <stdbool.h>

#define KEY_QUEUE_SIZE 8 


extern volatile uint8_t key_last[4] ;
typedef enum {
    KEY_EVENTUP = 0,
    KEY_EVENTDOWN,
    KEY_EVENTENTER,
    KEY_EVENTBACK
} key_name;

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
uint8_t key_read(GPIO_Ports_ENUM key, uint8_t index);
void KeyProcess(key_name key);

#endif