/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\adc_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-08-23 23:34:09
 */

#include "util_adc.h"

uint32_t ADC_ValueBuf[30]; // Adc data array
float ADC_Voltage;  // Adc decode data

/**
 * @brief      Adc peripheral initialization
 * @param      NULL
 * @retval     NULL
 */
void ADC_Init()
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // Adc calibration
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC_ValueBuf, 1); // start Adc DMA,Get the first group data.
}

/**
 * @brief      Decode Adc data
 * @param      NULL
 * @retval     NULL
 */
void ADC_Decode()
{
    ADC_Voltage = (float)ADC_ValueBuf[0] / 65535.0f * 11.0f * 2.9f; // adc decode 24V AVCC
}
