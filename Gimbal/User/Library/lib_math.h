/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-21 18:11:33
 */

#ifndef ALG_MATH_H
#define ALG_MATH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "arm_math.h"
#include "stdlib.h"
#include "stm32g4xx_hal.h"

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LimitMaxMin(input, max, min) \
    {                                \
        if (input > max)             \
        {                            \
            input = max;             \
        }                            \
        else if (input < min)        \
        {                            \
            input = min;             \
        }                            \
    }

#define GetMaxandMinRange(input, max, min) \
    {                                      \
        if (input >= max)                  \
        {                                  \
            max = input;                   \
        }                                  \
        else if (input <= min)             \
        {                                  \
            min = input;                   \
        }                                  \
    }

    typedef struct
    {
        float acc;
        float dec;
    } Math_SlopeParamTypeDef;

    float Math_RadToAngle(float rad);
    float Math_AngleToRad(float angle);
    float Math_Fal(float e, float alpha, float zeta);
    int16_t Math_Fsg(float x, float d);
    int16_t Math_Sign(float Input);
    float Math_InvSqrt(float x);
    void Math_InitSlopeParam(Math_SlopeParamTypeDef *pparam, float acc, float dec);
    float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef *pparam);
    float Math_CalcAbsSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef *pparam);
    float Math_Differential(float arr[], uint8_t order);

#ifdef __cplusplus
}
#endif

#endif
