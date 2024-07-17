/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:16:13
 */

#include "alg_pid.h"
#include "lib_math.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf,
              float sum_max, float output_max, float kd_cutoff_frq, float kf_cutoff_frq)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = 0.001f;
    pid->kf = kf;
    pid->sum_max = sum_max;
    pid->output_max = output_max;
    Filter_Lowpass_Init(kd_cutoff_frq, &(pid->d_filter));
    Filter_Lowpass_Init(kf_cutoff_frq, &(pid->kf_filter));
}

void PID_Clear(PID_TypeDef *pid)
{
    pid->sum = 0;
    pid->output_fdf = 0;
    pid->output = 0;
}

float PID_Calc(PID_TypeDef *pid)
{
    float dError, Error, ref_dError;
    Error = pid->ref - pid->fdb;
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = Error;
    dError = Math_Differential(pid->err, 1);
    pid->err_fdf[2] = pid->err_fdf[1];
    pid->err_fdf[1] = pid->err_fdf[0];
    pid->err_fdf[0] = pid->ref;
    ref_dError = Math_Differential(pid->err_fdf, 1);

    pid->sum = pid->sum + pid->ki * 0.5 * (pid->err[0] + pid->err[1]) * pid->dt;
    LimitMax(pid->sum, pid->sum_max);
    pid->output_fdf = Filter_Lowpass((pid->kf * ref_dError), &(pid->kf_filter));
    pid->output = pid->kp * Error + pid->sum + pid->kd * Filter_Lowpass(dError / pid->dt, &(pid->d_filter));
    pid->output += pid->output_fdf;
    LimitMax(pid->output, pid->output_max);

    return pid->output;
}

void PID_SetRef(PID_TypeDef *pid, float ref)
{
    pid->ref = ref;
}

void PID_SetFdb(PID_TypeDef *pid, float fdb)
{
    pid->fdb = fdb;
}
