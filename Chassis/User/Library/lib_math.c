/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-01-10 11:21:19
 */

#include "lib_math.h"
#include "cmsis_os.h"

#define RESPONSE_MODE FREQ_RESPONSE

// Add these three variables to watch
float openloop_spd_ref = 0.0f;
float openloop_spd_fdb = 0.0f;
uint8_t start_ident_flag = 0;

static void Process_Freq(float freq)
{
    int tick = 0;
    float t_ms = round(1000. / freq);
    freq = 1000. / t_ms;
    while(tick < (20 * t_ms))
    {
        openloop_spd_ref = 6000 * sin(freq * 2 * PI * tick * 0.001f);
        tick++;
        osDelay(1);
    }
}

static void Process_Step(float coef)
{
    int tick = 0;
    while(tick < 1000)
    {
        openloop_spd_ref = coef * 1000;
        tick++;
        osDelay(1);
    }
}

float step_coef = 2.0f;
void Test_Response(void)
{
#if RESPONSE_MODE == FREQ_RESPONSE
    for(float freq = 1.0f; freq <= 10.0f; freq += 0.5f)
    {
        Process_Freq(freq);
    }
#endif

#if RESPONSE_MODE == STEP_RESPONSE
    for(float times = 2.0f; times <= 8.0f; times += 0.5f)
    {
        step_coef = times;
        Process_Step(step_coef);
        step_coef *= -1;
        Process_Step(step_coef);
    }
#endif
}

float Math_Normalize(float input, float minInput, float maxInput)
{
    float modulus = maxInput - minInput;
    float normalized_input = input;
    int numMax = (int)((normalized_input - minInput) / modulus);
    normalized_input -= numMax * modulus;

    int numMin = (int)((normalized_input - maxInput) / modulus);
    normalized_input -= numMin * modulus;
    return normalized_input;
}

float Math_Consequent_To_180(float angle_ref, float angle_fdb)
{
    return angle_ref - Math_Normalize(angle_ref - angle_fdb, -180.0f, 180.0f);
}

float Math_Consequent_To_Limited(float angle_rad_ref, float angle_rad_fdb)
{
    return angle_rad_ref - Math_Normalize(angle_rad_ref - angle_rad_fdb, -PI, PI);
}

/**
 * @brief      Radian to angle
 * @param      Converted radian
 * @retval     result
 */
float Math_RadToAngle(float rad)
{
    return (rad * 180.0f / PI);
}

/**
 * @brief      Angle to Radian
 * @param      Converted radian
 * @retval     result
 */
float Math_AngleToRad(float angle)
{
    return (angle * PI / 180.0f);
}

/**
 * @brief      Continuous power function with linear segment near the origin
 * @param      NULL
 * @retval     result
 */
float Math_Fal(float e, float alpha, float zeta)
{
    int16_t s = 0;
    float fal_output = 0;
    s = (Math_Sign(e + zeta) - Math_Sign(e - zeta)) / 2;
    fal_output = e * s / (powf(zeta, 1 - alpha)) + powf(fabs(e), alpha) * Math_Sign(e) * (1 - s);
    return fal_output;
}

/**
 * @brief      Calculate fsg
 * @param      x :Number to be calc
 * @retval     result
 */
int16_t Math_Fsg(float x, float d)
{
    int16_t output = 0;
    output = (Math_Sign(x + d) - Math_Sign(x - d)) / 2;
    return output;
}

/**
 * @brief      Positive and negative judgment function
 * @param      x :Number to be judged
 * @retval     Positive output 1, negative output - 1, otherwise output 0
 */
int16_t Math_Sign(float x)
{
    int16_t output = 0;
    if(x > 0)
    {
        output = 1;
    }
    else if(x < 0)
    {
        output = -1;
    }
    else
        output = 0;
    return output;
}

/**
 * @brief      This shit is used to calculate the quick square root
 * @param      x :Number of square root
 * @retval     One third of the open results
 */
float Math_InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief      Calculation differential (only two order)(To be improved)
 * @param      arr :point to be differential value
 * @param      order :The differential order
 * @retval     NULL
 */
float Math_Differential(float arr[], uint8_t order)
{
    float value;
    switch(order)
    {
    case 1:
        value = arr[0] - arr[1];
        break;
    case 2:
        value = arr[2] - 2 * arr[1] + arr[0];
        break;
    default:
        value = arr[0];
        break;
    }
    return value;
}

/**
 * @brief      Initialize ramp function control parameters
 * @param      pparam: Pointer to ramp function control parameter
 * @param      kp: P factor
 * @param      ki: I factor
 * @param      kd: D factor
 * @param      sum_max: Integral limiting
 * @param      output_max: Output limiting
 * @retval     NULL
 */
void Math_InitSlopeParam(Math_SlopeParamTypeDef* pparam, float acc, float dec)
{
    pparam->acc = acc;
    pparam->dec = dec;
}

/**
 * @brief      Calculate slope function setting
 * @param      rawref: Current setting value
 * @param      targetref: Target set point
 * @param      pparam: Pointer to ramp function control parameter
 * @retval     Slope function setting value. If slope function is not enabled (parameter is 0), the target setting value is returned
 */
float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam)
{
    float newref;
    if(pparam->acc == 0 | pparam->dec == 0)
        return targetref;
    if(rawref < targetref - pparam->acc)
    {
        newref = rawref + pparam->acc;
    }
    else if(rawref > targetref + pparam->dec)
    {
        newref = rawref - pparam->dec;
    }
    else
    {
        newref = targetref;
    }
    return newref;
}

/**
 * @brief      Calculate the absolute slope function setting value
 * @param      rawref: Current setting value
 * @param      targetref: Target set point
 * @param      pparam: Pointer to ramp function control parameter
 * @retval     Absolute value ramp function setting value. If ramp function is not enabled, the target setting value is returned
 */
float Math_CalcAbsSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam)
{
    float newref;
    if(pparam->acc == 0 | pparam->dec == 0)
        return targetref;
    if(rawref > 0)
    {
        if(rawref < targetref - pparam->acc)
        {
            newref = rawref + pparam->acc;
        }
        else if(rawref > targetref + pparam->dec)
        {
            newref = rawref - pparam->dec;
        }
        else
        {
            newref = targetref;
        }
    }
    else
    {
        if(rawref > targetref + pparam->acc)
        {
            newref = rawref - pparam->acc;
        }
        else if(rawref < targetref - pparam->dec)
        {
            newref = rawref + pparam->dec;
        }
        else
        {
            newref = targetref;
        }
    }
    return newref;
}
