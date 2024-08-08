/*
 * @Project: Infantry Code
 *
 * @Author: Q
 * @Date: 2024-07-03 17:12:00
 * @LastEditors: Q
 * @LastEditTime: 2024-07-03 17:12:00
 */
 
#include "app_monitor.h"

/**
 * @brief      
 * @param      
 * @retval     NULL
 */

char ErrorCode_Watch[3] = {'N','0','0'};
void Write_ErrorCode_Watch(char type, uint8_t id, uint8_t errCode){
	ErrorCode_Watch[0] = type;
	ErrorCode_Watch[1] = '0'+id;
	ErrorCode_Watch[2] = '0'+errCode;
}
void beep_init(){
	//HAL_TIM_Base_Start_IT(&htim8);
	//HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
}
void beep_once(uint16_t tms1, uint16_t tms2){
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 100);//Start_Beep
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	osDelay(tms1);
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);//Stop_Beep
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	osDelay(tms2);
}
void Monitor_Init(){
	beep_init();
	beep_once(20,30);
}
void error_beep(int8_t times1,int8_t times2){
	for(;times1 > 0;times1 --){
		beep_once(20,50);
	}
	osDelay(150);
	for(;times2 > 0;times2 --){
		beep_once(20,50);
	}
	osDelay(300);
}
uint8_t Check_Type3508(Motor_DataTypeDef* pmotor){
	if(Motor_IsLost(pmotor)){
		pmotor->state = MOTOR_LOST;
	}else{
		pmotor->state = MOTOR_CONNECTED;
	}
	if(Motor_IsLostEncoderUpdate(pmotor)){//no encoder
		if(pmotor->state == MOTOR_LOST){	//no can
			return CAN_LOST_ERR;
		}else{	//no 7pin
			return SEVEN_PIN_LOST_ERR;
		}
	}
	return 0;
}
uint8_t Check_Type6020(Motor_DataTypeDef* pmotor){
	if(Motor_IsLost(pmotor)){
		pmotor->state = MOTOR_LOST;
	}else{
		pmotor->state = MOTOR_CONNECTED;
	}
	if(pmotor->state == MOTOR_LOST){	//no can
		return CAN_LOST_ERR;
	}
	if(pmotor->encoder.temp > 80){	//
		return OVER_TEMP_ERR;
	}
	return 0;
}
uint8_t Check_TypeSnail(uint8_t NOT_OUTPUT){
  Shoot_ControlTypeDef *shooter = Shoot_GetControlPtr();
	static uint32_t last_not_DIED_ERR_time = 0;
	if(shooter->shoot_left.ref > 10.0f && ADC_Voltage > 15.0f
		&& (Motor_shooterMotorLeft.encoder.speed < 3.0f || Motor_shooterMotorRight.encoder.speed < 3.0f)){
			if(HAL_GetTick() - last_not_DIED_ERR_time > 200 && !NOT_OUTPUT){
				return SNAIL_DIED_ERR;
			}
	}else{
		last_not_DIED_ERR_time = HAL_GetTick();
	}
	return 0;
}


void Monitor_Check(){
	uint8_t exist_err = 0,ErrorCode = 0;
	ErrorCode = Check_Type6020(Motor_gimbalMotors.motor_handle[1]);
	if(ErrorCode != 0){
		Write_ErrorCode_Watch('P',2,ErrorCode);
//		error_beep(2,ErrorCode);
		exist_err++;
	}
	ErrorCode = Check_Type3508(Motor_feederMotors.motor_handle[0]);
	if(ErrorCode != 0){
		Write_ErrorCode_Watch('F',1,ErrorCode);
//		error_beep(3,ErrorCode);
		exist_err++;
	}
	ErrorCode = Check_TypeSnail(exist_err);
	if(ErrorCode != 0){
		Write_ErrorCode_Watch('S',0,ErrorCode);
//		error_beep(1,ErrorCode);
		exist_err++;
	}
	
	if(exist_err == 0){
		Write_ErrorCode_Watch('N',0,0);
	}
}