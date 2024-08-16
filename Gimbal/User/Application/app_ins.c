/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-02 22:59:20
 */
// INS全称为 inertial navigation system，即惯性导航系统。此文件用于获得准确的imu角度。
#include "app_ins.h"

INS_DataTypeDef INS_Data;
INS_DataTypeDef *INS_GetControlPtr()
{
    return &INS_Data;
}

// 卡尔曼滤波参数初始化
void INS_Init()
{
    INS_DataTypeDef *ins = INS_GetControlPtr();
    ins->ins_param.scale[X_AXIS] = 1;
    ins->ins_param.scale[Y_AXIS] = 1;
    ins->ins_param.scale[Z_AXIS] = 1;
    ins->ins_param.yaw = 0;
    ins->ins_param.pitch = 0;
    ins->ins_param.roll = 0;
    ins->ins_param.init_flag = 0;
    QEKF_Init(&(ins->q), 10, 0.01f, 10000000, 1, 0.0085f);
}

// imu数据更新
void INS_Upadte(BMI088_DataTypeDef *bmi088)
{
    INS_DataTypeDef *ins = INS_GetControlPtr();
    static uint32_t count = 0;
    ins->update_dt = DWT_GetDeltaT(&ins->last_update_tick);
    if (((count % 1) == 0))
    {
        BMI088_Decode(bmi088);
        QEKF_Update(&(ins->q), bmi088->gyro.pitch, bmi088->gyro.row, bmi088->gyro.yaw,
                    bmi088->accel.x, bmi088->accel.y, bmi088->accel.z, ins->update_dt);
        ins->pitch = ins->q.pitch * Correction_Matrix[0] + ins->q.roll * Correction_Matrix[1] + ins->q.yaw * Correction_Matrix[2];
        ins->roll = ins->q.pitch * Correction_Matrix[3] + ins->q.roll * Correction_Matrix[4] + ins->q.yaw * Correction_Matrix[5];
        ins->yaw = ins->q.pitch * Correction_Matrix[6] + ins->q.roll * Correction_Matrix[7] + ins->q.yaw * Correction_Matrix[8];
        ins->yaw_consequent = ins->q.yaw_consequent;
        ins->accel[X_AXIS] = bmi088->accel.x * Correction_Matrix[0] + bmi088->accel.y * Correction_Matrix[1] + bmi088->accel.z * Correction_Matrix[2];
        ins->accel[Y_AXIS] = bmi088->accel.x * Correction_Matrix[3] + bmi088->accel.y * Correction_Matrix[4] + bmi088->accel.z * Correction_Matrix[5];
        ins->accel[Z_AXIS] = bmi088->accel.x * Correction_Matrix[6] + bmi088->accel.y * Correction_Matrix[7] + bmi088->accel.z * Correction_Matrix[8];
        ins->gyro[PITCH] = bmi088->gyro.pitch * Correction_Matrix[0] + bmi088->gyro.row * Correction_Matrix[1] + bmi088->gyro.yaw * Correction_Matrix[2];
        ins->gyro[ROLL] = bmi088->gyro.pitch * Correction_Matrix[3] + bmi088->gyro.row * Correction_Matrix[4] + bmi088->gyro.yaw * Correction_Matrix[5];
        ins->gyro[YAW] = bmi088->gyro.pitch * Correction_Matrix[6] + bmi088->gyro.row * Correction_Matrix[7] + bmi088->gyro.yaw * Correction_Matrix[8];
    }
    count++;
}
