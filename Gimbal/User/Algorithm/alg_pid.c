/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-02 22:45:39
 */

#include "alg_pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf, 
	          float sum_max, float output_max, float kd_cutoff_frq, float kf_cutoff_frq)
{
    pid->pid_mode = PID_POSITION;
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
    if (pid->pid_mode == PID_POSITION)
    {
        // Calculate the difference
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

        // Calculate the integral and integra anti-windup
        pid->sum = pid->sum + pid->ki * 0.5 * (pid->err[0] + pid->err[1]) * pid->dt;
        LimitMax(pid->sum, pid->sum_max);

        // Calculation results kf1_filter
        pid->output_fdf = Filter_Lowpass((pid->kf * ref_dError), &(pid->kf_filter));
        pid->output = pid->kp * Error + pid->sum + pid->kd * Filter_Lowpass(dError / pid->dt, &(pid->d_filter));
        pid->output += pid->output_fdf;

        // Output limiting
        LimitMax(pid->output, pid->output_max);
    }

    else if (pid->pid_mode == PID_DELTA)
    {
        // Calculate the difference
        float dError, ddError, Error, ref_dError, ref_ddError;
        Error = pid->ref - pid->fdb;
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        pid->err[0] = Error;
        dError = Math_Differential(pid->err, 1);
        ddError = Math_Differential(pid->err, 2);

        pid->err_fdf[2] = pid->err_fdf[1];
        pid->err_fdf[1] = pid->err_fdf[0];
        pid->err_fdf[0] = pid->ref;

        ref_dError = Math_Differential(pid->err_fdf, 1);
        ref_ddError = Math_Differential(pid->err_fdf, 2);

        // Calculate the integral and integral anti-windup
        if (pid->kp == 0)
        {
            pid->sum = Error;
        }
        else
        {
            pid->sum = Error + pid->err_lim / pid->kp;
        }
        LimitMax(pid->sum, pid->sum_max);

        // Calculation results kf1_filter
        pid->output_fdf = Filter_Lowpass((pid->kf * ref_dError), &(pid->kf_filter));
        pid->output = pid->kp * dError + pid->ki * pid->sum + pid->kd * Filter_Lowpass(ddError, &(pid->d_filter));
        pid->output += pid->output_fdf;

        // Output limiting
        float temp_limit = pid->output;
        LimitMax(pid->output, pid->output_max);
        pid->err_lim = pid->output - temp_limit;
    }
    return pid->output;
}

float PID_GetRef(PID_TypeDef *pid)
{
    return pid->ref;
}

void PID_SetRef(PID_TypeDef *pid, float ref)
{
    pid->ref = ref;
}

void PID_AddRef(PID_TypeDef *pid, float inc)
{
    pid->ref += inc;
}

float PID_GetFdb(PID_TypeDef *pid)
{
    return pid->fdb;
}

void PID_SetFdb(PID_TypeDef *pid, float fdb)
{
    pid->fdb = fdb;
}

float PID_GetOutput(PID_TypeDef *pid)
{
    return pid->output;
}

uint8_t Kp_RuleList[7][7] = {
    {PB, PB, PM, PM, PM, ZO, ZO},
    {PB, PM, PM, PM, PS, ZO, NS},
    {PM, PM, PS, PS, ZO, NS, NS},
    {PM, PS, PS, ZO, NS, NS, NM},
    {PS, PS, ZO, NS, NS, NM, NM},
    {PS, ZO, NS, NM, NM, NM, NB},
    {ZO, ZO, NM, NM, NM, NB, NB}};

uint8_t Ki_RuleList[7][7] = {
    {NB, NB, NM, NM, NS, ZO, ZO},
    {NB, NM, NM, NS, NS, ZO, ZO},
    {NB, NM, NS, NS, ZO, PS, PS},
    {NM, NM, NS, ZO, PS, PM, PM},
    {NM, NS, ZO, PS, PS, PM, PB},
    {ZO, ZO, PS, PS, PM, PM, PB},
    {ZO, ZO, PS, PM, PM, PB, PB}};

uint8_t Kd_RuleList[7][7] = {
    {PB, NS, NB, NB, NB, NM, PS},
    {PS, PM, NB, NM, NM, NS, ZO},
    {ZO, NS, PS, NM, NS, NS, ZO},
    {ZO, NS, NS, ZO, NS, NS, ZO},
    {ZO, ZO, ZO, ZO, NS, ZO, ZO},
    {PB, NS, PS, PS, PS, NM, PB},
    {PB, PM, PM, PM, PS, PS, NB}};

void FuzzyPID_Init(FuzzyPID_TypeDef *fuzzy_pid,
                   float kp_set[7], float ki_set[7], float kd_set[7],
                   Interval *error_range, Interval *error_change_range,
				   float kf, float sum_max, float output_max, float d_cutoff_frq, float kf_cutoff_frq)
{
	fuzzy_pid->kp_rule = Kp_RuleList;
    fuzzy_pid->ki_rule = Ki_RuleList;
    fuzzy_pid->kd_rule = Kd_RuleList;
    fuzzy_pid->kp_set = kp_set;
    fuzzy_pid->ki_set = ki_set;
    fuzzy_pid->kd_set = kd_set;
	fuzzy_pid->error_range = *error_range;
    fuzzy_pid->error_change_range = *error_change_range;
	
	fuzzy_pid->dt = 0.001f;
	fuzzy_pid->kf = kf;
    fuzzy_pid->sum_max = sum_max;
    fuzzy_pid->output_max = output_max;
	Filter_Lowpass_Init(d_cutoff_frq, &(fuzzy_pid->d_filter));
	Filter_Lowpass_Init(kf_cutoff_frq, &(fuzzy_pid->kf_filter));
}

float FuzzyPID_Calc(FuzzyPID_TypeDef *fuzzy_pid)
{
	float Error;
    Error = fuzzy_pid->ref - fuzzy_pid->fdb;
    fuzzy_pid->err[1] = fuzzy_pid->err[0];
	fuzzy_pid->err[0] = Error;
	fuzzy_pid->derror = Math_Differential(fuzzy_pid->err, 1);
	
	fuzzy_pid->err_fdf[1] = fuzzy_pid->err_fdf[0];
    fuzzy_pid->err_fdf[0] = fuzzy_pid->ref;
	fuzzy_pid->ref_derror = Math_Differential(fuzzy_pid->err_fdf, 1);
	
	fuzzy_pid->kp = TableLookup(fuzzy_pid->kp_rule, fuzzy_pid->kp_set, &(fuzzy_pid->error_range), &(fuzzy_pid->error_change_range), Error, fuzzy_pid->derror);
	fuzzy_pid->ki = TableLookup(fuzzy_pid->ki_rule, fuzzy_pid->ki_set, &(fuzzy_pid->error_range), &(fuzzy_pid->error_change_range), Error, fuzzy_pid->derror);
	fuzzy_pid->kd = TableLookup(fuzzy_pid->kd_rule, fuzzy_pid->kd_set, &(fuzzy_pid->error_range), &(fuzzy_pid->error_change_range), Error, fuzzy_pid->derror);

    fuzzy_pid->sum = fuzzy_pid->sum + fuzzy_pid->ki * 0.5 * (fuzzy_pid->err[0] + fuzzy_pid->err[1]) * fuzzy_pid->dt;
    LimitMax(fuzzy_pid->sum, fuzzy_pid->sum_max);

    fuzzy_pid->output_fdf = Filter_Lowpass((fuzzy_pid->kf * fuzzy_pid->ref_derror), &(fuzzy_pid->kf_filter));
    fuzzy_pid->output = fuzzy_pid->kp * Error + fuzzy_pid->sum + fuzzy_pid->kd * Filter_Lowpass(fuzzy_pid->derror / fuzzy_pid->dt, &(fuzzy_pid->d_filter));
    fuzzy_pid->output += fuzzy_pid->output_fdf;

    LimitMax(fuzzy_pid->output, fuzzy_pid->output_max);
	return fuzzy_pid->output;
}
		
void FuzzyPID_SetRef(FuzzyPID_TypeDef *fuzzy_pid, float ref)
{
    fuzzy_pid->ref = ref;
}

void FuzzyPID_SetFdb(FuzzyPID_TypeDef *fuzzy_pid, float fdb)
{
    fuzzy_pid->fdb = fdb;
}

float P, Pc;
float TableLookup(const uint8_t (*rule)[7], float *set, Interval *eRange, Interval *ecRange, float e, float ec)
{
    float mappedE = 3 * (2 * e - eRange->right - eRange->left) / (eRange->right - eRange->left);
    float mappedEC = 3 * (2 * e - ecRange->right - ecRange->left) / (ecRange->right - ecRange->left);
	P = mappedE;
	Pc = mappedEC;
    uint8_t eIndex1, eIndex2;
    float eWeight;
    uint8_t ecIndex1, ecIndex2;
    float ecWeight;
    if (mappedE < -3)
    {
        eIndex1 = NB;
        eIndex2 = NB;
        eWeight = 0;
    }
    else if (mappedE < -2)
    {
        eIndex1 = NB;
        eIndex2 = NM;
        eWeight = mappedE + 3;
    }
    else if (mappedE < -1)
    {
        eIndex1 = NM;
        eIndex2 = NS;
        eWeight = mappedE + 2;
    }
    else if (mappedE < 0)
    {
        eIndex1 = NS;
        eIndex2 = ZO;
        eWeight = mappedE + 1;
    }
    else if (mappedE < 1)
    {
        eIndex1 = ZO;
        eIndex2 = PS;
        eWeight = mappedE;
    }
    else if (mappedE < 2)
    {
        eIndex1 = PS;
        eIndex2 = PM;
        eWeight = mappedE - 1;
    }
    else if (mappedE < 3)
    {
        eIndex1 = PM;
        eIndex2 = PB;
        eWeight = mappedE - 2;
    }
    else
    {
        eIndex1 = PB;
        eIndex2 = PB;
        eWeight = 0;
    }

    if (mappedEC < -3)
    {
        ecIndex1 = NB;
        ecIndex2 = NB;
        ecWeight = 0;
    }
    else if (mappedEC < -2)
    {
        ecIndex1 = NB;
        ecIndex2 = NM;
        ecWeight = mappedEC + 3;
    }
    else if (mappedEC < -1)
    {
        ecIndex1 = NM;
        ecIndex2 = NS;
        ecWeight = mappedEC + 2;
    }
    else if (mappedEC < 0)
    {
        ecIndex1 = NS;
        ecIndex2 = ZO;
        ecWeight = mappedEC + 1;
    }
    else if (mappedEC < 1)
    {
        ecIndex1 = ZO;
        ecIndex2 = PS;
        ecWeight = mappedEC;
    }
    else if (mappedEC < 2)
    {
        ecIndex1 = PS;
        ecIndex2 = PM;
        ecWeight = mappedEC - 1;
    }
    else if (mappedEC < 3)
    {
        ecIndex1 = PM;
        ecIndex2 = PB;
        ecWeight = mappedEC - 2;
    }
    else
    {
        ecIndex1 = PB;
        ecIndex2 = PB;
        ecWeight = 0;
    }
    return set[rule[eIndex1][ecIndex1]] * (1 - eWeight) * (1 - ecWeight) + set[rule[eIndex1][ecIndex2]] * (1 - eWeight) * ecWeight + set[rule[eIndex2][ecIndex1]] * eWeight * (1 - ecWeight) + set[rule[eIndex2][ecIndex2]] * eWeight * ecWeight;
}
