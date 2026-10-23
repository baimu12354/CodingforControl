#include "NEUQ_Main.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

#include "menu.h"
uint8_t clear=0;
// extern uint8_t ret;
volatile uint8_t lcd_fresh =0;
extern float gyro_x_bias, gyro_y_bias, gyro_z_bias;
extern uint16_t calibration_count;
void Lcd_Draw(void)
{
// 0.02 0.01 -0.01
// 118.76 32.62 -12.15

    MenuNode *temp;
    uint8_t line = 0;
    char text[32];


    MenuRefresh();
    // if(lastselected != selected && lastselected != NULL)

    // {
    //     NEUQ_LCD200_ClearRect(8, 12, 12, 320);  
    //     lastselected = selected;      

    // }
    if(clear==1)
    {    
        NEUQ_LCD200_ClearRect(0, 38, 320, 320);
        clear=0;
    }

    if(calibration_count==1000)
    {
    sprintf(text, "%.2f  %.2f  %.2f", gyro_x_bias, gyro_y_bias, gyro_z_bias);
    NEUQ_LCD200_DisplayString(8,170, text);
    }
    // sprintf(text, "%d",ret);
    // NEUQ_LCD200_DisplayString(8,170, text);
    NEUQ_LCD200_DisplayString(8, 8, (char *)page->info->name);
    if(page == &nodes[MENU_RAW])
    {
        NEUQ_LCD200_DisplayString(10,10,"ACC(g)  GYRO(0/s)");


    }
    if(page == &nodes[MENU_ATTITUDE])
    {
        NEUQ_LCD200_DisplayString(10,10,"angle(0/s) fresh (10ms)");
    }
    temp = page->child;
    if (temp == NULL) return;
    do 
    {
        sprintf(text, "%c %-13s", temp == selected ? (editing ? '*' : '>') : ' ', temp->info->name);
        NEUQ_LCD200_DisplayString(8, (uint16_t)(38 + line * 28), text);
        if (temp->info->value != NULL) 
        {
            if(temp->info== &menu_table[MENU_ROLL] || temp->info== &menu_table[MENU_PITCH] || temp->info== &menu_table[MENU_YAW])
            {
                sprintf(text, "%5.1f", *temp->info->value);
                NEUQ_LCD200_DisplayString(142, (uint16_t)(38 + line * 28), text);
            }
            else
            {
                float value = (temp == selected && editing) ? edit_value : *temp->info->value;
                sprintf(text, "%8.2f", value);
                NEUQ_LCD200_DisplayString(142, (uint16_t)(38 + line * 28), text);
            }
        }
        temp = temp->next;
        line++;
    } while (temp != page->child );
    // NEUQ_LCD200_DisplayString(8, 280, "UP/DN MOVE  EN OK  BK BACK");
}