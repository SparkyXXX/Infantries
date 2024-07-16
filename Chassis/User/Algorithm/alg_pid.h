/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-07-06 21:11:35
 */

#ifndef ALG_PID_H
#define ALG_PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lib_math.h"
#include "lib_filter.h"

    typedef enum
    {
        PID_POSITION = 0u,
        PID_DELTA = 1u
    } PID_ModeEnum;

    typedef struct
    {
        PID_ModeEnum pid_mode;
        float kp;
        float ki;
        float kd;
		float dt;
        float kf;
        float sum_max;
        float output_max;
        Filter_Lowpass_TypeDef d_filter;
        Filter_Lowpass_TypeDef kf_filter;

        float ref;
        float fdb;
        float sum;
        float output;

        float err[3];
        float err_fdf[3]; // Feedforard
        float err_lim;    // Integral anti-windup
        float output_fdf; // Feedforard output
    } PID_TypeDef;

    void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf,
	          float sum_max, float output_max, float kd_cutoff_frq, float kf_cutoff_frq);
    void PID_Clear(PID_TypeDef *pid);
    float PID_Calc(PID_TypeDef *pid);
    void PID_SetRef(PID_TypeDef *pid, float ref);
    void PID_SetFdb(PID_TypeDef *pid, float fdb);
#ifdef __cplusplus
}
#endif

#endif
