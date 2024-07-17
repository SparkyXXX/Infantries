/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:23:29
 */

#ifndef PERIPH_CAP_H
#define PERIPH_CAP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

    typedef struct
    {
        uint8_t SD_flag;
        uint8_t ui_state;
        float sum_power;
        uint8_t rest_energy;
        uint32_t last_update_time;
    } Cap_DataTypeDef;

    Cap_DataTypeDef *Cap_GetDataPtr(void);
    void Cap_Init(void);
    void Cap_Update(void);

#ifdef __cplusplus
}
#endif

#endif
