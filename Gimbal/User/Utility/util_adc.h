/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Utility\adc_util.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-18 02:33:01
 */

#ifndef ADC_UTIL_H
#define ADC_UTIL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "adc.h"
#include "stdint.h"

    extern uint32_t ADC_ValueBuf[30];
    extern float ADC_Voltage;

    void ADC_Init(void);
    void ADC_Decode(void);

#endif

#ifdef __cplusplus
}
#endif
