/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-04-19 23:25:35
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

    typedef enum
    {
        NB = 0,
        NM = 1,
        NS = 2,
        ZO = 3,
        PS = 4,
        PM = 5,
        PB = 6,
    } FuzzyPID_TableEnum;

    typedef struct
    {
        float left;
        float right;
    } Interval;

    typedef struct
    {
        float kp;
        float ki;
        float kd;
		float kf;
		float dt;
        float sum_max;
        float output_max;

        const uint8_t (*kp_rule)[7];
        const uint8_t (*ki_rule)[7];
        const uint8_t (*kd_rule)[7];
        float *kp_set;
        float *ki_set;
        float *kd_set;

        Interval error_range;
        Interval error_change_range;
        Filter_Lowpass_TypeDef d_filter;
        Filter_Lowpass_TypeDef kf_filter;

        float ref;
        float fdb;
        float sum;
        float output;

		float derror;
		float ref_derror;
		float err[2];
        float err_fdf[2]; // Feedforard
        float output_fdf; // Feedforard output

    } FuzzyPID_TypeDef;

	void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf,
	          float sum_max, float output_max, float kd_cutoff_frq, float kf_cutoff_frq);
    void PID_Clear(PID_TypeDef *pid);
    float PID_Calc(PID_TypeDef *pid);
    void PID_SetRef(PID_TypeDef *pid, float ref);
    void PID_SetFdb(PID_TypeDef *pid, float fdb);

	void FuzzyPID_Init(FuzzyPID_TypeDef *fuzzy_pid,
                   float kp_set[7], float ki_set[7], float kd_set[7],
                   Interval *error_range, Interval *error_change_range,
				   float kf, float sum_max, float output_max, float d_cutoff_frq, float kf_cutoff_frq);
    float FuzzyPID_Calc(FuzzyPID_TypeDef *fuzzy_pid);
    void FuzzyPID_SetRef(FuzzyPID_TypeDef *fuzzy_pid, float ref);
    void FuzzyPID_SetFdb(FuzzyPID_TypeDef *fuzzy_pid, float fdb);
    float DeFuzzy(const uint8_t (*rule)[7], float *set, Interval *eRange, Interval *ecRange, float e, float ec);

#ifdef __cplusplus
}
#endif

#endif
