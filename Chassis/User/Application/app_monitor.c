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
void beep_once(uint16_t tms){
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 100);//Start_Beep
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	osDelay(tms);
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);//Stop_Beep
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	osDelay(tms);
}
void Monitor_Init(){
	beep_init();
	beep_once(50);
}
void error_beep(int8_t times1,int8_t times2){
	for(;times1 > 0;times1 --){
		beep_once(100);
	}
	osDelay(400);
	for(;times2 > 0;times2 --){
		beep_once(100);
	}
	osDelay(800);
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

void Monitor_Check(){
	uint8_t i = 0,exist_err = 0,ErrorCode = 0;
	
	for(i=0;i<4;i++){
		ErrorCode = Check_Type3508(Motor_ChassisMotors.motor_handle[i]);
		if(ErrorCode != 0){
			Write_ErrorCode_Watch('M',i+1,ErrorCode);
//			error_beep(i+1,CAN_LOST_ERR);
			exist_err++;
		}
	}
	ErrorCode = Check_Type6020(Motor_GimbalMotors.motor_handle[0]);
	if(ErrorCode != 0){
		Write_ErrorCode_Watch('Y',2,ErrorCode);
//		error_beep(5,ErrorCode);
		exist_err++;
	}
	if(BoardCom_IsLost(BOARDCOM_PKG_CTRL)||BoardCom_IsLost(BOARDCOM_PKG_IMU)||
	BoardCom_IsLost(BOARDCOM_PKG_CHA_REF)||BoardCom_IsLost(BOARDCOM_PKG_UI_STATE)){
		Write_ErrorCode_Watch('B',0,0);
//		error_beep(6,6);
		exist_err++;
	}
	if(BoardCom_IsLost(BOARDCOM_PKG_CAP1)){
		Write_ErrorCode_Watch('C',0,0);
//		error_beep(7,7);
		exist_err++;
	}
	if(Referee_IsLost()){
		Write_ErrorCode_Watch('R',0,0);
//		error_beep(8,8);
		exist_err++;
	}
	
	if(exist_err == 0){
		Write_ErrorCode_Watch('N',0,0);
	}
}