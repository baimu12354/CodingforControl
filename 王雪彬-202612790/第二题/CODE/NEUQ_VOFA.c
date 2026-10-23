#include "NEUQ_lib_include.h"


int NEUQ_VOFA_fmt_float(char *out, float v, int dec){
    int neg = (v < 0.0f);
    float av = neg ? -v : v;
    float scale = 1.0f;
    for(int i = 0; i < dec; i++) scale *= 10.0f;

    long fixed = (long)(av * scale + 0.5f);   // 四舍五入后的定点整数
    long ip = fixed / (long)scale;            // 整数部分
    long fp = fixed % (long)scale;            // 小数部分

    int len = 0;
    if(neg) out[len++] = '-';
    len += snprintf(out + len, 16, "%ld", ip);
    out[len++] = '.';
    for(int i = dec - 1; i >= 0; i--){
        out[len + i] = (char)('0' + fp % 10);
        fp /= 10;
    }
    len += dec;
    out[len] = '\0';
    return len;
}

// VOFA+ FireWater 数据引擎
void NEUQ_VOFA_send_frame(const float *ch, uint8_t ch_count){
    char buf[96];
    int len = 0;
    for(uint8_t i = 0; i < ch_count; i++){
        if(i > 0) buf[len++] = ',';
        len += NEUQ_VOFA_fmt_float(buf + len, ch[i], 3);   // 3 位小数
    }
    buf[len++] = '\r';
    buf[len++] = '\n';
    NEUQ_Uart_write_data(UART_0, (uint8_t*)buf, (uint32_t)len);
}
