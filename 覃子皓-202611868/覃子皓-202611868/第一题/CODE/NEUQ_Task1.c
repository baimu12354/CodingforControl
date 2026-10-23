#include "NEUQ_lib_include.h"


#define LCD_LINE_CURRNT   0
#define LCD_LINE_PENDING  40
#define LCD_LINE_VOLITAGE   80

static volatile uint8_t current_scheme = 0;
static volatile uint8_t pending_scheme = 0;
static volatile uint16_t seg_time_ms = 0;
static volatile uint8_t seg_index = 0;
//static volatile float battery_voltage = 0.0f;

typedef struct {
    uint16_t duration_ms;  //持续时间
    uint8_t level;   //电平
}Led_segment;

typedef struct {
    const Led_segment *segments;  //段
    uint8_t       seg_count;  //段数
}Led_scheme;

static const Led_segment scheme1[] = {
    {1000, 1},
    {1000, 0},
};
static const Led_segment scheme2[] = {
    {200,  1},
    {1800, 0},
};
static const Led_segment scheme3[] = {
    {200, 1},
    {200, 0},
    {200, 1},
    {1400, 0},
};
static const Led_segment scheme4[] = {
    {200, 1},
    {200, 0},
    {600, 1},
    {1000, 0},
};
static const Led_scheme schemes[] = {
    {scheme1, 2},
    {scheme2, 2},
    {scheme3, 4},
    {scheme4, 4},
};

/*static float NEUQ_Task1_adc_to_voltage(uint16_t adc_val){
    //#define DIVIDER_RATIO 4.0f
    float v_PA3 = ((float)adc_val / 4095.0f) * 3.3f;
    return v_PA3 ;//* DIVIDER_RATIO;
}*/

void NEUQ_Task1_init(void){
    NEUQ_LED_init();
    NEUQ_Key_init();
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    //adc_init(ADC1_CH3_PA3,ADC_12BIT);
    current_scheme = 0;
    pending_scheme = 0;
    seg_index      = 0;
    seg_time_ms    = 0; 
    NEUQ_LED_on();   
}

void NEUQ_Task1_task(void){
    char buf[32];
    if (NEUQ_Key_scan()){
        pending_scheme = (pending_scheme + 1) % 4;  //1到4循环
    }
    /*{
        uint16_t adc_val = adc_mean_filter_convert(ADC1_CH3_PA3, 5);
        battery_voltage = NEUQ_Task1_adc_to_voltage(adc_val);
    }*/
    sprintf(buf,"Current : %d",current_scheme + 1);  //读取并存储
    NEUQ_LCD200_DisplayString(0, LCD_LINE_CURRNT, buf);

    sprintf(buf,"Pending : %d", pending_scheme +1);
    NEUQ_LCD200_DisplayString(0, LCD_LINE_PENDING, buf);

    /*sprintf(buf,"voltage : %.2f V",battery_voltage);
    NEUQ_LCD200_DisplayString(0, LCD_LINE_VOLITAGE, buf);*/


}

void NEUQ_Task1_tick_1ms(void){
    const Led_scheme *sch = &schemes[current_scheme];
    const Led_segment *seg = &sch->segments[seg_index];

    seg_time_ms++;
    if(seg_time_ms >= seg->duration_ms){  //判断当前段是否完成
        seg_time_ms = 0;
        seg_index++;  //下一段
        if (seg_index >= sch->seg_count){  //重复方案每一段
            seg_index = 0;
            if (pending_scheme !=current_scheme){  //改变方案
                current_scheme = pending_scheme;
                sch = &schemes[current_scheme];
                seg = &sch->segments[seg_index];

            }
        }else{  //保证每个方案都完整执行
            seg = &sch->segments[seg_index];
        }
    }
    if (seg->level){
        NEUQ_LED_on();
    }else{
        NEUQ_LED_off();
    }

}