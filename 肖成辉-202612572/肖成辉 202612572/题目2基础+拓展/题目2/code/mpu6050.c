#include "mpu6050.h"









//0ax 1ay 2az 3gx .......

DATA datatable[6]={
    {&mpu6050_acc_x,0,0,0,{0.0f,0.0f,0.0f}},
    {&mpu6050_acc_y,0,0,0,{0.0f,0.0f,0.0f}},
    {&mpu6050_acc_z,0,0,0,{0.0f,0.0f,0.0f}},
    {&mpu6050_gyro_x,0,0,0,{0.0f,0.0f,0.0f}},
    {&mpu6050_gyro_y,0,0,0,{0.0f,0.0f,0.0f}},
    {&mpu6050_gyro_z,0,0,0,{0.0f,0.0f,0.0f}},
    };

void LPF_Init(void) {

   
    for (uint8_t i = 0; i < 6; i++)
    {
    datatable[i].name.alpha  = 0.1;
    datatable[i].name.y= 0.0f;
    datatable[i].name.inited = 0;
    }
    
    
}

void DataGet(void){NEUQ_MPU6050_read_data();
   for (uint8_t i = 0; i < 6; i++)
   {
    if (i < 3)
    {
        datatable[i].rawdata = MPU6050_acc_transition(*datatable[i].value);
    }else{
        datatable[i].rawdata = MPU6050_gyro_transition(*datatable[i].value);
    }

    datatable[i].name.y = datatable[i].name.alpha*datatable[i].rawdata+(1.0f-datatable[i].name.alpha)*datatable[i].name.y;
    datatable[i].fl_data = datatable[i].name.y;
   }

   
}









void VOFA(void){
    int16_t i = param.axis - 1;
    if (i < 0) i = 0;

    if (param.start)
    {
        if (datatable[i].enable)
        {
            printf("%.2f,%.2f\n",datatable[i].fl_data,datatable[i].rawdata);
        
        }
        else{printf("%.2f\n",datatable[i].rawdata);}
        
    }
    
}





