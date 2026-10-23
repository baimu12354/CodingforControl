float battery_voltage(void)
{
    return ((float)adc_mean_filter_convert(ADC_BATTERY, 5) / 4095.0f)
           * ADC_VREF * ADC_DIVIDER;
}
