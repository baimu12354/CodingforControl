#include "math.h"

float Roll;
float Pitch;
float Yaw;
 float q0 = 1.0f;
 float q1 = 0.0f;
 float q2 = 0.0f;
 float q3 = 0.0f;
 float eintx=0;
 float einty=0;
 float eintz=0;

float Kp_e = 2.0f;
float Ki_e = 0.005f;
//开平方根
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1); 
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y)); 
    return y;
}



void fournumber_update(float ax, float ay, float az, float gx, float gy, float gz) {


    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    gx
    *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;


    ex= (ay * vz - az * vy);
    ey= (az * vx - ax * vz);
    ez= (ax * vy - ay * vx);

    eintx+=ex*0.01;
    einty+=ey*0.01;
    eintz+=ez*0.01;


    gx+= Kp_e*ex+eintx*Ki_e;
    gy+= Kp_e*ey+einty*Ki_e;
    gz+=Kp_e*ez+eintz*Ki_e;


    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;

 
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) *0.005;
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) *0.005;
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) *0.005;
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) *0.005;


    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;


    Roll = atan2f(2.0f * (q0 * q1 + q2 * q3),1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2957795f;



    float temp = 2.0f * (q0 * q2 - q1 * q3);


    if 
    (fabsf(temp) >= 1.0f)
    {
        Pitch = temp>0?90.0:-90.0;
    }
    else
    Pitch = asinf(temp) * 57.2957795f;



     Yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f;







}