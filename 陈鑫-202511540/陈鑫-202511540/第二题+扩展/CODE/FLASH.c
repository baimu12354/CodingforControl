#include "NEUQ_lib_include.h"

void flash_param_load(void){
    uint8_t buf[sizeof(FILTER_PARAMS)];
    if(NEUQ_Flash_read(PARAM_SECTOR,buf,sizeof(FILTER_PARAMS))!=NEUQ_FLASH_OK)
    {
            filter_init();
            return;
    }
    memcpy(&g_filter_param,buf,sizeof(FILTER_PARAMS));

    uint16_t sum=0;
    for(int i=0;i<sizeof(FILTER_PARAMS)-2;i++)
    {
        sum+=buf[i];
    }
    if(sum!=g_filter_param.checksum){
        filter_init();
    }

}

uint8_t flash_param_save(void){
    uint8_t buf[sizeof(FILTER_PARAMS)];

    g_filter_param.checksum=0;
    memcpy(buf,&g_filter_param,sizeof(FILTER_PARAMS));
    uint16_t sum=0;
    for(int i=0;i<sizeof(FILTER_PARAMS)-2;i++)
    {
        sum+=buf[i];
    }
    g_filter_param.checksum=sum;
    memcpy(buf,&g_filter_param,sizeof(FILTER_PARAMS));
    vofa_suspend();   // 保存前挂起 vofa，防止 PIT2 触发发送
    uint8_t res=NEUQ_Flash_save(PARAM_SECTOR,buf,sizeof(FILTER_PARAMS));
    vofa_resume();    // 保存完恢复

    return res;
    
}