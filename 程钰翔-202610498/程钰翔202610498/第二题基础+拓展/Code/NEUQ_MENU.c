#include "NEUQ_lib_include.h"
#include "NEUQ_MENU.h"

 extern volatile uint8_t  current_key_id; 
 uint8_t current_page=PAGE_MAIN;      //初始化当前菜单是主菜单
 uint8_t current_choice=0;            //初始化当前选项是0选项，我规定的三个选项是0，1，2
 uint8_t current_choice_new=0;
 uint8_t param_change_choice=0;       //二级菜单中修改参数，0时无法修改，点击按键in变为1，可以修改
 float target_coefficient=0.2f;       //为啥加f，这是滤波系数,初始值定为0.2       如果无法编译就加extern在.h里
 float gyro_x_filtered=0;             //x轴滤波后的值
 float gyro_y_filtered=0;             //y轴滤波后的值
 float gyro_z_filtered=0;             //z轴滤波后的值
 float gyro_x_real=0;                 //x轴原始数据
 float gyro_y_real=0;                 //y轴原始数据
 float gyro_z_real=0;                 //z轴原始数据
 float acc_x_filtered=0;             //x轴滤波后的值
 float acc_y_filtered=0;             //y轴滤波后的值
 float acc_z_filtered=0;             //z轴滤波后的值
 float acc_x_real=0;                 //x轴原始数据
 float acc_y_real=0;                 //y轴原始数据
 float acc_z_real=0;                 //z轴原始数据
 uint8_t target_send_state=0;         //传送状态 0为停止发送 1为开始发送
 uint8_t target_axis=0;               //0为x轴  1为y轴  2为x轴
 //float out_real=0,out_filtered=0;     //最终的输出的滤波值，有局部变量了



//int32_t param[6]={MPU6050_acc_transition(mpu6050_acc_x ),MPU6050_acc_transition(mpu6050_acc_y ),MPU6050_acc_transition(mpu6050_acc_z ),MPU6050_acc_transition(mpu6050_gyro_x ),MPU6050_acc_transition(mpu6050_gyro_y ),MPU6050_acc_transition(mpu6050_gyro_z ),}             //当前还没给陀螺仪等读出数据的变量，先用a，b，c表示
                  /*char *main_menu_items[3]{
                      "6_Axis Data"       //六轴数据显示
                      "Wave Filter"       //波形与滤波设置
                      "Attitude"          //姿态解算
                  }
                  */  //第二种方法但是不知道为啥过不了编译
void Menu_Display(void){
    if(current_page==PAGE_MAIN)/*主菜单*/{
      NEUQ_LCD200_DisplayString(10,10,"MAIN  PAGE");
      if(current_choice==0){
        NEUQ_LCD200_DisplayString(10,50,">  6_Axis Data");  //六轴数据显示
        NEUQ_LCD200_DisplayString(10,90,"   Wave Filter");  //波形与滤波设置
        NEUQ_LCD200_DisplayString(10,130,"   Attitude");    //姿态解算
      }
      else if(current_choice==1){
        NEUQ_LCD200_DisplayString(10,50,"   6_Axis Data");
        NEUQ_LCD200_DisplayString(10,90,">  Wave Filter");
        NEUQ_LCD200_DisplayString(10,130,"   Attitude");
      }
      else if(current_choice==2){
        NEUQ_LCD200_DisplayString(10,50,"   6_Axis Data");
        NEUQ_LCD200_DisplayString(10,90,"   Wave Filter");
        NEUQ_LCD200_DisplayString(10,130,">  Attitude");
      }
     /*
      for(int i=0;i<3;i++){
        if(current_choice==i){
          NEUQ_LCD200_DisplayString(10,50+40*i,">  ");
        }else{
          NEUQ_LCD200_DisplayString(10,50+40*i,"  ");
        }
       NEUQ_LCD200_DisplayString(30,50+40*i,main_menu_items[i]);
       //第二次修改的方案比第一次好很多，但是不知道为啥过不了编译
    }
       */
  }else if(current_page==PAGE_NEXT){
        if(current_choice==0){
          NEUQ_LCD200_DisplayString(10,10,"6_Axis Data");
          NEUQ_LCD200_DisplayString(10,40,"acc_x:");
          NEUQ_LCD200_DisplayString(10,70,"acc_y:");
          NEUQ_LCD200_DisplayString(10,100,"acc_z:");
          NEUQ_LCD200_DisplayString(10,130,"gyro_x:");
          NEUQ_LCD200_DisplayString(10,160,"gyro_y:");
          NEUQ_LCD200_DisplayString(10,190,"gyro_z:");
          NEUQ_LCD200_DisplayFloat(100,40,MPU6050_acc_transition(mpu6050_acc_x ),6,2);
          NEUQ_LCD200_DisplayFloat(100,70,MPU6050_acc_transition(mpu6050_acc_y ),6,2);
          NEUQ_LCD200_DisplayFloat(100,100,MPU6050_acc_transition(mpu6050_acc_z ),6,2);
          NEUQ_LCD200_DisplayFloat(100,130,MPU6050_gyro_transition(mpu6050_gyro_x ),5,2);
          NEUQ_LCD200_DisplayFloat(100,160,MPU6050_gyro_transition(mpu6050_gyro_y ),5,2);
          NEUQ_LCD200_DisplayFloat(100,190,MPU6050_gyro_transition(mpu6050_gyro_z),5,2); //printf("%.2f,%.2f,%.2f\n"MPU6050_acc_transition(mpu6050_acc_x ), MPU6050_acc_transition(mpu6050_acc_y), MPU6050_acc_transition(mpu6050_acc_z),MPU6050_gyro_transition(mpu6050_gyro_x),MPU6050_gyro_transition(mpu6050_gyro_y),MPU6050_gyro_transition(mpu6050_gyro_z));
        //陀螺仪需要单位的转换(角速度的单位是°/s，加速度的单位是g)
         NEUQ_LCD200_DisplayString(160,40,"g");
         NEUQ_LCD200_DisplayString(160,70,"g");
         NEUQ_LCD200_DisplayString(160,100,"g");
         NEUQ_LCD200_DisplayString(170,130,"degrees/s");
         NEUQ_LCD200_DisplayString(170,160,"degrees/s");
         NEUQ_LCD200_DisplayString(170,190,"degrees/s");
      }else if(current_choice==1){
          NEUQ_LCD200_DisplayString(10,10,"Wave Filter");
          //current_choice_new=0;       //下一个函数可能也要写
          if(current_choice_new==0){       //呈现行数
            if(param_change_choice==0){
              NEUQ_LCD200_DisplayString(10,40,"> Axis: ");
            }else{
               NEUQ_LCD200_DisplayString(10,40,">> Axis:");
            }
          }else{
             NEUQ_LCD200_DisplayString(10,40," Axis:"); 
          }

         // if(target_axis==0)    NEUQ_LCD200_DisplayString(60,40,"X");
         // if(target_axis==1)    NEUQ_LCD200_DisplayString(60,40,"Y");
         // if(target_axis==2)    NEUQ_LCD200_DisplayString(60,40,"Z");

          if(current_choice_new==1){     //呈现系数
            if(param_change_choice==0){
              NEUQ_LCD200_DisplayString(10,70,"> coefficient:");
            }else{
              NEUQ_LCD200_DisplayString(10,70,">> coefficient:");
            }
          }else{
            NEUQ_LCD200_DisplayString(10,70," coefficient:");
           }

         // NEUQ_LCD200_DisplayFloat(100,70,target_coefficient,4,2);
          
          if(current_choice_new==2){    //呈现发送或者关闭
            if(param_change_choice==0){
              NEUQ_LCD200_DisplayString(10,100,"> Send:");
            }else{
              NEUQ_LCD200_DisplayString(10,100,">> Send:");
            }
          }else{
            NEUQ_LCD200_DisplayString(10,100," Send:");
          }

          if(target_send_state==1)  {
            NEUQ_LCD200_DisplayString(120,100,"ON");
          }else  NEUQ_LCD200_DisplayString(120,100,"OFF");
          NEUQ_LCD200_DisplayFloat(200,70,target_coefficient,4,2);
          if(target_axis==0)    NEUQ_LCD200_DisplayString(120,40,"gyro_x");
          if(target_axis==1)    NEUQ_LCD200_DisplayString(120,40,"gyro_y");
          if(target_axis==2)    NEUQ_LCD200_DisplayString(120,40,"gyro_z");
          if(target_axis==3)    NEUQ_LCD200_DisplayString(120,40,"acc_x");
          if(target_axis==4)    NEUQ_LCD200_DisplayString(120,40,"acc_y");
          if(target_axis==5)    NEUQ_LCD200_DisplayString(120,40,"acc_z");
        }
        else if(current_choice==2){
          NEUQ_LCD200_DisplayString(10,10,"Attitude");
          NEUQ_LCD200_DisplayString(10,50,"Roll:");   //roll是翻滚角
          NEUQ_LCD200_DisplayFloat(100,50,roll,6,2);
          NEUQ_LCD200_DisplayString(10,90,"Pitch:");  //pitch是俯仰角
          NEUQ_LCD200_DisplayFloat(100,90,pitch,6,2);
          NEUQ_LCD200_DisplayString(10,130,"Yaw:");   //yaw是偏航角
          NEUQ_LCD200_DisplayFloat(100,130,yaw,6,2);
        }
  }
}
    
void MENU_Act(void){
    if(current_key_id!=0)/*按键按下*/{
            if(current_page==PAGE_MAIN)/*第0页编辑*/{
                if(current_key_id==1)/*up 键*/{
                    if(current_choice>0){current_choice--;}
                    else{current_choice=2;}
                }else if(current_key_id==2)/*down键*/{
                        if(current_choice<2){current_choice++;}
                        else{current_choice=0;}
                }else if(current_key_id==4)/*in键*/{
                        current_page=PAGE_NEXT;
                        param_change_choice=0;
                        NEUQ_LCD200_Clear();
                        Menu_Display();

                    }
            }else if(current_page==PAGE_NEXT)/*第1页编辑*/{
                    NEUQ_LCD200_ClearRect(100,40,150,150);//清除从(100,40)开始，宽150，高150的区域
                   // NEUQ_MPU6050_read_data() ;
                     if(current_key_id==3){    //back键
                      if(param_change_choice==0){
                        current_page=PAGE_MAIN;
                        param_change_choice=0;  /*可以去掉*/
                        NEUQ_LCD200_Clear();
                      }else{
                        param_change_choice=0;
                        NEUQ_Flash_write_sector(511,(uint8_t *)&target_coefficient,4);
                      
                      }
                    }else if(current_key_id==1)/*up 键*/{
                      if(param_change_choice==0){
                        if(current_choice_new>0){current_choice_new--;}
                        else{current_choice_new=2;}
                      }else{
                        if(current_choice_new==0){
                          if(target_axis>0){target_axis--;}
                          else{target_axis=5;}
                        }else if(current_choice_new==1){
                          if(target_coefficient<1.0){target_coefficient+=0.05f;}
                          else{target_coefficient=0.0;}
                        } else if(current_choice_new==2){
                           target_send_state=!target_send_state;
                        }
                      }
                   // param_change_choice=0;
                }else if(current_key_id==2)/*down键*/{
          
                        if(param_change_choice==0){
                        if(current_choice_new<2){current_choice_new++;}
                        else{current_choice_new=0;}
                      }else{
                        if(current_choice_new==0){
                          if(target_axis<5){target_axis++;}
                          else{target_axis=0;}
                        }else if(current_choice_new==1){
                          if(target_coefficient>0.0){target_coefficient-=0.05f;}
                          else{target_coefficient=1.0;}
                        } else if(current_choice_new==2){
                          target_send_state=!target_send_state;
                        }
                      }
                }else if(current_key_id==4)/*in键*/{
                  if(param_change_choice==0&&current_choice==1){
                    param_change_choice=1;
                  }
                        //param_change_choice=1;
                        NEUQ_LCD200_Clear();
                        Menu_Display();/*可以删掉*/
                }
                current_key_id=0;
               // Menu_Display();
            }
            current_key_id=0;
            if(current_page==PAGE_NEXT){
                 Menu_Display();/*刷新*/
                } NEUQ_Delay_ms(50);
    
  }
}

//简化main函数
void MPU_Data_Update(void){
        NEUQ_MPU6050_read_data() ;// 读取MPU6050传感器数据，数据保存在 MPU6050_Data_get 结构体中
        float gx = MPU6050_gyro_transition(mpu6050_gyro_x);
        float gy = MPU6050_gyro_transition(mpu6050_gyro_y);
        float gz = MPU6050_gyro_transition(mpu6050_gyro_z);
        float ax = MPU6050_acc_transition(mpu6050_acc_x);
        float ay = MPU6050_acc_transition(mpu6050_acc_y);
        float az = MPU6050_acc_transition(mpu6050_acc_z);
        gyro_x_real=gx-gyro_x_offset;
        gyro_y_real=gy-gyro_y_offset;
        gyro_z_real=gz-gyro_z_offset;
        acc_x_real=ax;
        acc_y_real=ay;
        acc_z_real=az;
}
void LowPassFilter_Update(void){
        gyro_x_filtered=LowPassFilter(gyro_x_real,gyro_x_filtered,target_coefficient);
        gyro_y_filtered=LowPassFilter(gyro_y_real,gyro_y_filtered,target_coefficient);
        gyro_z_filtered=LowPassFilter(gyro_z_real,gyro_z_filtered,target_coefficient);
        acc_x_filtered=LowPassFilter(acc_x_real,acc_x_filtered,target_coefficient);
        acc_y_filtered=LowPassFilter(acc_y_real,acc_y_filtered,target_coefficient);
        acc_z_filtered=LowPassFilter(acc_z_real,acc_z_filtered,target_coefficient);
}
void Target_Send(void){
        if(target_send_state==1){
            float out_real=0,out_filtered=0;
            if(target_axis==0)   {out_real=gyro_x_real;out_filtered=gyro_x_filtered;}
            else if(target_axis==1)   {out_real=gyro_y_real;out_filtered=gyro_y_filtered;}
            else if(target_axis==2)   {out_real=gyro_z_real;out_filtered=gyro_z_filtered;}
            else if(target_axis==3)   {out_real=acc_x_real;out_filtered=acc_x_filtered;}
            else if(target_axis==4)   {out_real=acc_y_real;out_filtered=acc_y_filtered;}
            else if(target_axis==5)   {out_real=acc_z_real;out_filtered=acc_z_filtered;}
            printf("%.2f,%.2f\n",out_real,out_filtered);//放在if里面，所以在off时没有数据包括0，打开on后才会有数据
        } 
      }
 
///////////////////////////////////////////////*以上是第一个选项“六轴数据显示”*///////////////////////////////////////

/*接下来是波形与滤波设置*/
float LowPassFilter(float  new_value,float old_value,float coefficient){
  return coefficient*new_value+(1.0f-coefficient)*old_value;
}   //计算一阶低通滤波函数

////////////////////陀螺仪零偏标定在 NEUQ 库/////////////////////
// 陀螺仪零偏的标定与静止自动跟踪现放到 NEUQ 库
//（Library/NEUQ_lib/NEUQ_lib_APP/NEUQ_IMU_Solve.c），供所有工程复用：
//   mpu_gyro_offset_calibrate()   上电静止标定零偏
//   mpu_gyro_offset_track(...)    静止时自动跟踪零偏，抑制 yaw 缓慢漂移
// 注意：加速度计不参与零偏标定（静止时含 1g 重力，减掉会破坏姿态解算的重力参考）。

