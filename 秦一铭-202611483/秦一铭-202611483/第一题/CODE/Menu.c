#include "NEUQ_lib_include.h"

void Menu_Show(void){
    static int last_cur = -1, last_pnd = -1;   // 初值 -1 保证首次必画

    /* 值没变就不刷，省掉无谓的SPI占用 */
    if ((current_mode == last_cur) && (pending_mode == last_pnd)){
        return;
    }
    last_cur = current_mode;
    last_pnd = pending_mode;

    char line[32];

    snprintf(line, sizeof(line), "Current: Mode %d", current_mode);
    NEUQ_LCD200_DisplayString(10, 10, line);

    snprintf(line, sizeof(line), "Pending: Mode %d", pending_mode);
    NEUQ_LCD200_DisplayString(10, 40, line);
}