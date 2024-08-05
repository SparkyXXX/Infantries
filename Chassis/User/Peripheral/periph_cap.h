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

#define CAP_OFFLINE_TIME 10

	typedef enum
    {
        CAP_CONNECTED = 0,
        CAP_LOST = 1
    } Cap_StateEnum;
	
    typedef struct
    {
		Cap_StateEnum state;
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
	void Cap_IsLost(void);

#ifdef __cplusplus
}
#endif

#endif
