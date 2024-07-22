/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:21:27
 */
#include "lib_power_ctrl.h"
#include "lib_ringbuff.h"
#include "math.h"

float PredictPower(const PCArgsType pp_args, const float I, const float w)
{
    const uint8_t SameSign = (I * w >= 0);
    const uint8_t Boundary = (w < 0.3 && w > -0.3);

    if (Boundary)
    {
        return 0.5 * (pp_args.mp * I * w + pp_args.kp[0] * I * I + pp_args.kp[1] * w * w + pp_args.lp * fabs(w) + pp_args.mn[0] * I * I * w * w + pp_args.mn[1] * I * w + pp_args.kn[0] * I * I + pp_args.kn[1] * w * w + pp_args.ln * fabs(w));
    }

    if (SameSign)
    {
        return (pp_args.mp * I * w + pp_args.kp[0] * I * I + pp_args.kp[1] * w * w + pp_args.lp * fabs(w));
    }
    else
    {
        return (
            pp_args.mn[0] * I * I * w * w + pp_args.mn[1] * I * w + pp_args.kn[0] * I * I + pp_args.kn[1] * w * w + pp_args.ln * fabs(w));
    }
}

float PowerControlGetCurrentP(const PCArgsType Arg, const float P, const float w)
{
    float Delta = sqrt(4 * (P * Arg.kp[0] - Arg.kp[0] * Arg.kp[1] * w * w - Arg.lp * Arg.kp[0] * fabs(w)) + Arg.mp * Arg.mp * w * w);
    if (w < 0)
    {
        return ((-Arg.mp * w - Delta) / (2 * Arg.kp[0]));
    }
    else
    {
        return ((-Arg.mp * w + Delta) / (2 * Arg.kp[0]));
    }
}

float PowerControlGetCurrentN(const PCArgsType Arg, const float P, const float w)
{
    float Delta = sqrt(
        4 * (P * Arg.kn[0] + Arg.mn[0] * w * w - Arg.kn[0] * Arg.kn[1] * w * w - Arg.kn[0] * Arg.ln * fabs(w) - Arg.kn[1] * Arg.mn[0] * w * w * w * w - Arg.ln * Arg.mn[0] * w * w * fabs(w)) + Arg.mn[1] * Arg.mn[1] * w * w);
    if (w < 0)
    {
        return ((-Arg.mn[1] * w + Delta) / 2 / (Arg.kn[0] + Arg.mn[0] * w * w));
    }
    else
    {
        return ((-Arg.mn[1] * w - Delta) / 2 / (Arg.kn[0] + Arg.mn[0] * w * w));
    }
}

void PowerControl(
    const PCArgsType Arg, const float PowerTarget, float CurrentToSend[4], const float SpeedNow[4])
{
    static sheriff::SlidingWindow<float, 16> MotorSpeedWindow[4];
    static float PCTestW[4] = {0};
    static float ChassisW[4] = {0};
    static float power[4];

    for (int i = 0; i < 4; i++)
    {
        PCTestW[i] -= MotorSpeedWindow[i][0];
        MotorSpeedWindow[i] << SpeedNow[i];
        PCTestW[i] += MotorSpeedWindow[i][15];

        ChassisW[i] = PCTestW[i] / 16.0f;
    }

    for (int i = 0; i < 4; i++)
    {
        power[i] = PredictPower(Arg, CurrentToSend[i], SpeedNow[i]);
    }

    if (power[0] + power[1] + power[2] + power[3] <= PowerTarget)
    {
        return;
    }

    static float TotalPowerPositive, TotalPowerNegative;
    TotalPowerPositive = 0;
    TotalPowerNegative = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (power[i] > 0)
        {
            TotalPowerPositive += power[i];
        }
        else
        {
            TotalPowerNegative += power[i];
        }
    }

    float k = (PowerTarget - TotalPowerNegative) / TotalPowerPositive;

    for (int i = 0; i < 4; i++)
    {
        if (CurrentToSend[i] * SpeedNow[i] >= 0)
        {
            CurrentToSend[i] = PowerControlGetCurrentP(Arg, power[i] * k, SpeedNow[i]);
        }
        //			else {
        //                CurrentToSend[i] = PowerControlGetCurrentN(Arg, power[i] * k, SpeedNow[i]);
        //            }
    }
}