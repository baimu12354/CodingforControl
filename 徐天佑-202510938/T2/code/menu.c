#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "NEUQ_Main.h"
#include "menu.h"






 float menu_value[17];
 MenuNode nodes[22];
 MenuNode *page;
 MenuNode *selected;
 MenuNode *lastselected;


 float edit_value;

 
 MenuInfo menu_table[] = 
 {
    

    [MENU_MAIN] = { "MPU6050 MENU", -1, NULL, 0 },
    [MENU_RAW] = { "six data ", MENU_MAIN, NULL, 0 },
    [MENU_FILTER] = { "filter ", MENU_MAIN, NULL, 0 },
    [MENU_ATTITUDE] = { "angle", MENU_MAIN, NULL, 0 },
    [MENU_RAW_AX] = { "ACC X", MENU_RAW, &menu_value[0], 0 },
    [MENU_RAW_AY] = { "ACC Y", MENU_RAW, &menu_value[1], 0 },
    [MENU_RAW_AZ] = { "ACC Z", MENU_RAW, &menu_value[2], 0 },
    [MENU_RAW_GX] = { "GYRO X", MENU_RAW, &menu_value[3], 0 },
    [MENU_RAW_GY] = { "GYRO Y", MENU_RAW, &menu_value[4], 0 },
    [MENU_RAW_GZ] = { "GYRO Z", MENU_RAW, &menu_value[5], 0 },
    [MENU_ALPHA_ACC] = { "ALPHA_ACC", MENU_FILTER, &filter_alfa_acc, 1 },
    [MENU_ALPHA_GYRO] = { "ALPHA_GYRO", MENU_FILTER, &filter_alfa_gyro, 1 },
    [MENU_VOFA_ENABLE] ={ "TANSMIT",MENU_FILTER, NULL, 0 },
    [MENU_AX] = { "ACC X",  MENU_VOFA_ENABLE, &menu_value[6], 0 },
    [MENU_AY] = { "ACC Y",  MENU_VOFA_ENABLE, &menu_value[7], 0 },
    [MENU_AZ] = { "ACC Z",  MENU_VOFA_ENABLE, &menu_value[8], 0 },
    [MENU_GX] = { "GYRO X", MENU_VOFA_ENABLE, &menu_value[9],  0},
    [MENU_GY] = { "GYRO Y", MENU_VOFA_ENABLE, &menu_value[10], 0 },
    [MENU_GZ] = { "GYRO Z", MENU_VOFA_ENABLE, &menu_value[11], 0 },
    [MENU_ROLL] = { "ROLL", MENU_ATTITUDE, &menu_value[12], 0 },
    [MENU_PITCH] = { "PITCH", MENU_ATTITUDE, &menu_value[13], 0 },
    [MENU_YAW] = { "YAW", MENU_ATTITUDE, &menu_value[14], 0 }
};

 uint8_t MenuNodeId(const MenuNode *node)
{
    return (uint8_t)(node - nodes);
}

void Menu_Init(void)
{

    uint8_t i;
    memset(nodes, 0, sizeof(nodes));
    for (i = 0; i < (uint8_t)(sizeof(menu_table) / sizeof(menu_table[0])); i++) {
        MenuNode *node = &nodes[i];
        node->info = &menu_table[i];
        if (menu_table[i].parent_id >= 0) 
        {
            MenuNode *parent = &nodes[menu_table[i].parent_id];
            node->parent = parent;
            if (parent->child == NULL) 
            {
                parent->child = node;
                node->prev = node;
                node->next = node;
            } 
            else 
            {
                MenuNode *first = parent->child;
                MenuNode *last = first->prev;
                last->next = node;
                node->prev = last;
                node->next = first;
                first->prev = node;
            }
        }
    }
    page = &nodes[MENU_MAIN];
    selected = page->child;
    lastselected =NULL;
    editing = 0;
}


void MenuRefresh(void)
{

    menu_value[0] =MPU6050_acc_transition(imudata_.acc_x_f); 
    menu_value[1] =MPU6050_acc_transition(imudata_.acc_y_f); 
    menu_value[2] =MPU6050_acc_transition(imudata_.acc_z_f);
    menu_value[3] = MPU6050_gyro_transition(imudata_.gyro_x_f); 
    menu_value[4] = MPU6050_gyro_transition(imudata_.gyro_y_f);
    menu_value[5] = MPU6050_gyro_transition(imudata_.gyro_z_f);
    menu_value[12] = imudata_.roll; 
    menu_value[13] = imudata_.pitch; 
    menu_value[14] = imudata_.yaw;

}
// 18

// -4

// 16.5


