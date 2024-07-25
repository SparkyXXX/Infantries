/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-25 19:50:29
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

        uint8_t buck_version;
        uint8_t boost_version;
        float buck_input_current_max;
        float buck_output_current_max;
        float boost_input_current_max;
        float boost_output_current_max;
    } Cap_DataTypeDef;

    extern const float Cap_Version_Table[4][3];

    Cap_DataTypeDef *Cap_GetDataPtr(void);
    void Cap_Init(void);
    void Cap_Update(void);

#ifdef __cplusplus
}
#endif

#endif
