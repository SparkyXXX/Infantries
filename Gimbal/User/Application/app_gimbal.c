#include "app_gimbal.h"

Gimbal_ControlTypeDef Gimbal_Control;

/**
 * @brief      Gimbal initialization offset and mode
 * @param      NULL
 * @retval     NULL
 */

void Gimbal_Init()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->mode_change_flag = 0;
    gimbal->last_mode = GIMBAL_NO_AUTO;
    gimbal->present_mode = GIMBAL_NO_AUTO;
    gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
    Motor_gimbalMotorPitch.encoder.angle = 4000;
    Gimbal_ParamInit();
}

/**
 * @brief      Pitch state control
 * @param      NULL
 * @retval     NULL
 */
float test_ref = 0.0f;
void Gimbal_PitchOutput()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    INS_DataTypeDef *ins = INS_GetControlPtr();
    Remote_DataTypeDef *remote = Remote_GetDataPtr();

    if (gimbal->mode_change_flag == 1)
    {
        // Clear PID if there is multiple PID param been used
        gimbal->mode_change_flag = 0;
    }
#if PITCH_MODE == PITCH_REMOTE
    FuzzyPID_SetRef(&(gimbal->pitch_pos), -gimbal->pitch_position_ref * 3.1415926 / 180);
#endif
#if PITCH_MODE == PITCH_STEP
	FuzzyPID_SetRef(&(gimbal->pitch_pos), -test_ref);
#endif
    FuzzyPID_SetFdb(&(gimbal->pitch_pos), -ins->pitch * 3.1415926 / 180);
    FuzzyPID_SetRef(&(gimbal->pitch_spd), FuzzyPID_Calc(&gimbal->pitch_pos));
    FuzzyPID_SetFdb(&(gimbal->pitch_spd), -ins->gyro[PITCH]);
    Motor_SetOutput(&Motor_gimbalMotorPitch, FuzzyPID_Calc(&(gimbal->pitch_spd)));
    Motor_CAN_SendGroupOutput(&Motor_gimbalMotors);
}

/**
 * @brief      Gets the pointer to the gimbal control object
 * @param      NULL
 * @retval     The pointer points to the gimbal control object
 */
Gimbal_ControlTypeDef *Gimbal_GetControlPtr()
{
    return &Gimbal_Control;
}

/**
 * @brief      Change Gimbal mode
 * @param      mode: Gimbal mode enum
 * @retval     NULL
 */
void Gimbal_ModeSet(Gimbal_ModeEnum mode)
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    gimbal->last_mode = gimbal->present_mode;
    gimbal->present_mode = mode;
    if (gimbal->last_mode != gimbal->present_mode)
    {
        gimbal->mode_change_flag = 1;
    }
}

/**
 * @brief      Yaw state control
 * @param      NULL
 * @retval     NULL
 */
void Gimbal_YawModeSet()
{
    Gimbal_ControlTypeDef *gimbal = Gimbal_GetControlPtr();
    switch (gimbal->present_mode)
    {
    case GIMBAL_NO_AUTO:
        gimbal->yaw_mode = GIMBAL_YAW_NO_AUTO;
        break;
    case GIMBAL_ARMOR:
        gimbal->yaw_mode = GIMBAL_YAW_ARMOR;
        break;
    case GIMBAL_BIG_ENERGY:
        gimbal->yaw_mode = GIMBAL_YAW_BIG_ENERGY;
        break;
    case GIMBAL_SMALL_ENERGY:
        gimbal->yaw_mode = GIMBAL_YAW_SMALL_ENERGY;
        break;
    }
}
