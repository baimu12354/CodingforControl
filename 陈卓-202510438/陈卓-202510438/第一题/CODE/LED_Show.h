#ifndef _LED_SHOW_H_
#define _LED_SHOW_H_
#define LED (PD12)


extern PAGE_CONTROL PAGE;
extern uint32_t count;

typedef struct { uint8_t  lvl;   uint16_t ms;   } Seg_t;     /* 一段：电平 + 时长 */
typedef struct { uint8_t  nseg;  Seg_t   seg[4]; } Scheme_t;

static const Scheme_t SCHEME[4] =
{
    /* 方案1 */ { 2, { {1, 1000}, {0, 1000} } },
    /* 方案2 */ { 2, { {1,  200}, {0, 1800} } },
    /* 方案3 */ { 4, { {1,  200}, {0,  200}, {1, 200}, {0, 1400} } },
    /* 方案4 */ { 4, { {1,  200}, {0,  200}, {1, 600}, {0, 1000} } },
};

void LED_Init(void);
void LED_Tick(void);

#endif