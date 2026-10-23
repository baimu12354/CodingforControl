#include "NEUQ_lib_include.h"
#include "NEUQ_MENU.h"

extern volatile uint8_t  g_cur_scheme; 
extern volatile uint8_t  g_pending_scheme; 


void MENU_Data(void){
    NEUQ_LCD200_Display_int32(110,10,g_cur_scheme+1,1);
    NEUQ_LCD200_Display_int32(260,10,g_pending_scheme+1,1);
     NEUQ_LCD200_DisplayString(10,10,"Current:");
    NEUQ_LCD200_DisplayString(180,10,"NEXT:");
    NEUQ_LCD200_DisplayString(10,50,"1:1000-1000");
    NEUQ_LCD200_DisplayString(10,90,"2:200-1800");
    NEUQ_LCD200_DisplayString(10,130,"3:200-200-200-1400");
    NEUQ_LCD200_DisplayString(10,170,"4:200-200-600-1000");

}