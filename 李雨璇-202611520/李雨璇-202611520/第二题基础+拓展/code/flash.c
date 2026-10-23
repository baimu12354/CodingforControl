#include "NEUQ_lib_include.h"

extern Filter_Param A_X;
extern Filter_Param A_Y;
extern Filter_Param A_Z;
extern Filter_Param G_X;
extern Filter_Param G_Y;
extern Filter_Param G_Z;

// 数据写入Flash
void Flash_Send(float *a, float *b, float *c, float *d, float *e, float *f){
    uint32_t data[6];
    memcpy(&data[0], a, 4);
    memcpy(&data[1], b, 4);
    memcpy(&data[2], c, 4);
    memcpy(&data[3], d, 4);
    memcpy(&data[4], e, 4);
    memcpy(&data[5], f, 4);

    NEUQ_Flash_clear_protection();
    NEUQ_Flash_sector_erase(NEUQ_FLASH_W25Q16_LAST_SECTOR_ID); 
    NEUQ_Flash_page_program(NEUQ_FLASH_W25Q16_LAST_SECTOR_ID,(uint8_t *)data,24);
}

// 读取Flash中的数据
void Flash_Read(void){
    uint8_t buf[24];

    NEUQ_Flash_read(0x08060000,buf,24);
    
    memcpy(&(A_X.a), buf + 0, 4);
    memcpy(&(A_Y.a), buf + 4, 4);
    memcpy(&(A_Z.a), buf + 8, 4);
    memcpy(&(G_X.a), buf + 12, 4);
    memcpy(&(G_Y.a), buf + 16, 4);
    memcpy(&(G_Z.a), buf + 20, 4);
}