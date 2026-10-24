#include "NEUQ_lib_include.h"
#include "motor.h"
#include "frame_parse.h"

extern NEUQ_FIFO_t uart_rx_fifo;
extern uint8_t     uart_rx_fifo_buf[128];

void User_Main(void) {
    NEUQ_Debug_init();
    NEUQ_Delay_init();
    NEUQ_Timer_init();
    NEUQ_FIFO_init(&uart_rx_fifo, uart_rx_fifo_buf, 128);

    motor_init();
    NEUQ_PIT_init_ms(PIT_CH1, 10);   // 1ms 控制周期
     
    uint32_t last_send = 0;
    uint32_t last_diag = 0;


    while (1) {
        //motor_setduty(5000);
        frame_parse_poll();
        param_apply_poll();

        uint32_t now = NEUQ_Get_Timer_Value_ms();

        

        /* 10ms 上传 JustFloat */
        if (now - last_send >= 10) {
            last_send = now;
            send_status_justfloat();
        }
    }
}