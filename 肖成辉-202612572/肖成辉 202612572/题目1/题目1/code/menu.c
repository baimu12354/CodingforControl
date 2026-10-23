#include "menu.h"

void LCDSHOW(void)
{   
    uint8_t nowflag = Getflag();
    uint8_t preflag = Getpflag();
    char line[32];
    sprintf(line,"nowflag:%d",nowflag);
    NEUQ_LCD200_DisplayString(1,1,line);
    sprintf(line,"preflag:%d",preflag);
    NEUQ_LCD200_DisplayString(1,32,line);
    sprintf(line,"ADC:%lu",mv);
    NEUQ_LCD200_DisplayString(1,64,line);
    
    
}