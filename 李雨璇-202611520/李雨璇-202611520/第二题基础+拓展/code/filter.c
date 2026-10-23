#include "filter.h"

Filter_Param A_X = {0.20f, 0};
Filter_Param A_Y = {0.20f, 0};
Filter_Param A_Z = {0.20f, 0};
Filter_Param G_X = {0.20f, 0};
Filter_Param G_Y = {0.20f, 0};
Filter_Param G_Z = {0.20f, 0};

Filter_Param *axis[6] = {
    &A_X,
    &A_Y,
    &A_Z,
    &G_X,
    &G_Y,
    &G_Z,
};

void Filter_Init(Filter_Param *param){
    param->history = 0.0f;
}

void Filter_Init_all(void) { 
    for (uint8_t i = 0; i <= 5; i++){
        Filter_Init(axis[i]);
        }
}

void A_Send(uint8_t a, float b){
    uint8_t index = a;
    axis[index]->a = b;
}

float Filter_Data(Filter_Param *param, float new){
    param->history = (1.0f - param->a) * param->history + param->a * new;
    return param->history;
}