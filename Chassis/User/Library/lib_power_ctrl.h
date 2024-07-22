/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:21:13
 */

#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float mp;
        float kp[2];
        float cp;
        float lp;
        float mn[2];
        float kn[2];
        float cn;
        float ln;
    } PCArgsType;

    typedef struct
    {
        float lf;
        float rf;
        float lb;
        float rb;
    } OmiWheelCurrentType;

    typedef struct
    {
        float lf;
        float rf;
        float lb;
        float rb;
    } OmiWheelSpeedType;

    float PredictPower(const PCArgsType pp_args, const float I, const float w);

    float PowerControlGetCurrentP(const PCArgsType Arg, const float P, const float w);

    float PowerControlGetCurrentN(const PCArgsType Arg, const float P, const float w);

    void PowerControl(
        const PCArgsType Arg, const float PowerTarget, float CurrentToSend[4], const float SpeedNow[4]);

#ifdef __cplusplus
};
#endif