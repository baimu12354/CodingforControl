#include "NEUQ_Main.h"
#include "pid.h"
#include "td.h"
float k_forward=0; 
float k_forward2=0;
extern TD_conctrol TD1;  
float v_filter=0;
void pid_init(PID *Speed)
{
    Speed->Kp = 0;
    Speed->Ki = 0;
    Speed->Kd = 0;
    Speed->currenterror = 0;
    Speed->lasterror = 0;
    Speed->integral = 0;
    Speed->derivative = 0;
    Speed->target = 0;
    Speed->actual = 0;
    Speed->output = 0;
    Speed->lastactual = 0;

}

void pid_update(PID *Speed)
{

    Speed->lasterror = Speed->currenterror;

    Speed->currenterror = Speed->target - Speed->actual;

    if((Speed->output>10000 && Speed->currenterror>0) || (Speed->output<-10000 && Speed->currenterror<0) || Speed->Ki==0) 
    {
        Speed->integral=Speed->integral;
    }

    else
    {
        Speed->integral += Speed->currenterror;
    }

        // if((Speed->target - Speed->actual)<-20 && Speed->integral > 0)

        // {
        //     Speed->integral=0;

        // }

    if (Speed->integral > (4000 )) Speed->integral = (4000 );

    else if (Speed->integral < -(4000)) Speed->integral = -(4000 );
    if(Speed->target==0) Speed->integral=0;

    Speed->derivative = Speed->lastactual - Speed->actual;
    Speed->lastactual = Speed->actual;
    Speed->output = Speed->Kp * Speed->currenterror +Speed->Ki * Speed->integral + Speed->Kd * Speed->derivative;
    // Speed->output = -Speed->output ;
    Speed->output += k_forward*Speed->target + k_forward2*TD_get_R2(&TD1);
    if (Speed->output > 10000) Speed->output = 10000;
    if (Speed->output < -10000) Speed->output = -10000;

}


PID Speed;
