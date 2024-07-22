/*
 * @Project: Hatrix_Robot
 *
 * @Author: Hatrix
 * @Date: 2023-11-07 14:28:30
 * @LastEditors: Hatrix 3113624526@qq.com
 * @LastEditTime: 2024-02-12 17:02:44
 */

#ifndef APP_INS_H
#define APP_INS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "config_ctrl.h"
#include "alg_qekf.h"
#include "periph_bmi088.h"
#include "sys_dwt.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define PITCH 0
#define ROLL 1
#define YAW 2

    typedef struct
    {
        uint8_t init_flag;
        float scale[3];
        float yaw;
        float pitch;
        float roll;
    } INS_ParamTypeDef;

    typedef struct
    {
        float roll;
        float pitch;
        float yaw;
        float yaw_consequent;

        float gyro[3];
        float accel[3];
        Quaternion_DataTypeDef q;
        INS_ParamTypeDef ins_param;

        float update_dt;
        uint32_t last_update_tick;
    } INS_DataTypeDef;

    extern const float Correction_Matrix[9];

    INS_DataTypeDef* INS_GetControlPtr(void);
    void INS_Init(void);
    void INS_Upadte(BMI088_DataTypeDef* bmi088);

#endif

#ifdef __cplusplus
}
#endif
