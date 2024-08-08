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
			uint8_t mode; // switch of cap
			uint8_t cap_mode_stall;
			uint8_t cap_mode_starting;
			uint16_t starting_time;
			uint8_t boost_mode;
			uint8_t ui_state;
			float sum_power;
			float sum_current;
			float voltage;
			uint8_t rest_energy; 
			uint8_t SD_flag;
			uint32_t last_update_time;
			
			uint8_t	BUCK_VERSION;//lzq
			uint8_t BOOST_VERSION;
			float BUCK_INPUT_CURRENT_MAX;
			float BUCK_OUTPUT_CURRENT_MAX;
			float BOOST_INPUT_CURRENT_MAX;
			float BOOST_OUTPUT_CURRENT_MAX;//lzq
    } Cap_DataTypeDef;

    Cap_DataTypeDef *Cap_GetDataPtr(void);
    void Cap_Init(void);
    void Cap_Update(void);
		
		extern const float Cap_Version_Table[4][3];//lzq
    
#ifdef __cplusplus
}
#endif

#endif
