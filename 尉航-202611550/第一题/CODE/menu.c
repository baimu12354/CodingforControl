#include "menu.h"

void LCDSHOW(void){
    uint8_t nowplan = Getnowplan();
    uint8_t nextplan = Getnextplan();
    char line[32];
    snprintf(line,sizeof(line),"nowplan:%d",nowplan);
    NEUQ_LCD200_DisplayString(1,8,line);
    snprintf(line,sizeof(line),"nextplan:%d",nextplan);
    NEUQ_LCD200_DisplayString(1,36,line);
    
};