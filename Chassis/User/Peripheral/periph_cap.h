/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-14 19:40:25
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
//        uint8_t mode; // switch of cap
//        uint8_t cap_mode_stall;
//        uint8_t cap_mode_starting;
//        uint16_t starting_time;
        uint8_t SD_flag;
        uint8_t ui_state;
        float sum_power;
//        float sum_current;
//        float voltage;
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
