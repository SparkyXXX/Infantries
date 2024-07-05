#include "app_ui.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "protocol_referee.h"
#include "lib_ui.h"
#include "util_uart.h"
#include "cmsis_os.h"

UI::Rectangle shooterRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220}, {1920 - 100 + 15, 1080 - 220 + 15}};

UI::Rectangle magRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220 - 75}, {1920 - 100 + 15, 1080 - 220 + 15 - 75}};

UI::Rectangle autoAimRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220 - 150}, {1920 - 100 + 15, 1080 - 220 + 15 - 150}};

UI::Rectangle capRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220 - 225}, {1920 - 100 + 15, 1080 - 220 + 15 - 225}};

UI::Rectangle homeWarningRec{
	UI::Color::Pink, 15, {300, 1080 - 250}, {300 + 15, 1080 - 250 + 15}};

constexpr uint16_t eCapProgressBarLeft = 360;
constexpr uint16_t eCapProgressBarTop = 851;
constexpr uint16_t eCapProgressBarRight = 645;
constexpr uint16_t eCapProgressBarBottom = 824;

UI::Rectangle eCapProgressBarBox{
	UI::Color::Yellow, 3, {eCapProgressBarLeft, eCapProgressBarTop}, {eCapProgressBarRight, eCapProgressBarBottom}};

UI::Rectangle eCapProgressBarCor{
	UI::Color::Green,
	15,
	{eCapProgressBarLeft + 9, eCapProgressBarTop - 9},
	{eCapProgressBarRight - 9, eCapProgressBarBottom + 9}};

UI::Int eCapProgressValue{
	UI::Color::Green, 2, {eCapProgressBarRight + 9, eCapProgressBarBottom + 24}, 20, 100};

UI::Float pitchValue{
	UI::Color::Black, 2, {1680 + 50, 540 + 8}, 20, 1.23456f};

UI::Float MecVelocity{
	UI::Color::Purple, 2, {960 + 50, 540 + 8 + 40}, 20, 1.23456f};

UI::Float MecAngularVelocity{
	UI::Color::Orange, 2, {960 + 50, 540 + 8 - 40}, 20, 1.23456f};

UI::Line frontSightMiddle{
	UI::Color::Black, 1, {960, 690}, {960, 390}};

UI::Line frontSightUp1{
	UI::Color::Black, 1, {885, 690}, {1035, 690}};

UI::Line frontSightUp2{
	UI::Color::Black, 1, {910, 640}, {1010, 640}};

UI::Line frontSightUp3{
	UI::Color::Black, 1, {935, 590}, {985, 590}};

UI::Line frontSightUp4{
	UI::Color::Black, 1, {945, 665}, {975, 665}};

UI::Line frontSightUp5{
	UI::Color::Black, 1, {945, 615}, {975, 615}};

UI::Line frontSightDown1{
	UI::Color::Black, 1, {885, 390}, {1035, 390}};

UI::Line frontSightDown2{
	UI::Color::Black, 1, {910, 440}, {1010, 440}};

UI::Line frontSightDown3{
	UI::Color::Black, 1, {935, 490}, {985, 490}};

UI::Line frontSightDown4{
	UI::Color::Black, 1, {945, 415}, {975, 415}};

UI::Line frontSightDown5{
	UI::Color::Black, 1, {945, 465}, {975, 465}};

/* 实时更新的图层 */

UI::Layer modeLayer{
	"MD", 8, {&shooterRec, &magRec, &autoAimRec, &capRec, &homeWarningRec}};

UI::Layer progressBars{
	"PB", 8, {&eCapProgressBarCor, &eCapProgressValue, &pitchValue}};

/* 只绘制，不修改的图层 */

UI::Layer constLayer{
	"CL", 8, {&eCapProgressBarBox, &frontSightUp4, &frontSightUp5, &frontSightDown4, &frontSightDown5}};

UI::Layer frontSight{
	"FS",
	8,
	{&frontSightMiddle, &frontSightUp1, &frontSightUp2, &frontSightUp3, &frontSightDown1, &frontSightDown2,
	 &frontSightDown3}};

UI::String smaString{"SMA", 8, UI::Color::Cyan, 25, 3, {1920 - 300, 1080 - 200}};
#define ShootMagAutoSTR "SHOOTER:\n\n    MAG:\n\n   AUTO:"

UI::String capString{"CAP", 8, UI::Color::Cyan, 25, 3, {1920 - 200, 1080 - 420}};
#define CapSTR "CAP:"

UI::String warningString{"WAR", 8, UI::Color::Cyan, 20, 2, {95, 1080 - 232}};
#define WarningSTR "  WARNING:"

/* 偶尔更新的图层 */
UI::String chassisModeString{"CMD", 8, UI::Color::Cyan, 25, 3, {1920 / 2 - 200, 200}};
#define chassisModeStop "CHASSIS:STOP"
#define chassisModeNormal "CHASSIS:NORMAL"
#define chassisModeGyro "CHASSIS:GYRO"

UI::String aimModeString{"AMD", 8, UI::Color::Cyan, 25, 3, {1920 / 2 - 200, 150}};
#define aimModeNoAuto "    AIM:NO_AUTO"
#define aimModeArmor "    AIM:ARMOR"
#define aimModeSmallBuff "    AIM:SMALL_BUFF"
#define aimModeBigBuff "    AIM:BIG_BUFF"

uint8_t UiBuffer[200];

UI_DataTypeDef UI_Data;
HP_DataTypeDef HP_Data;

UI_DataTypeDef *UI_GetDataPtr()
{
	return &UI_Data;
}

HP_DataTypeDef *HP_GetDataPtr()
{
	return &HP_Data;
}

void ChangeModeStr(UI::String &uis, const char *str, uint16_t len)
{
	UI_DataTypeDef *ui = UI_GetDataPtr();
	uis.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, str, len);
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(50);
}

void UI_Init(void)
{
	UI_DataTypeDef *ui = UI_GetDataPtr();
	modeLayer.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	progressBars.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	constLayer.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	frontSight.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);

	smaString.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(""));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	capString.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(""));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	warningString.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(""));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	chassisModeString.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(""));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	aimModeString.getInitBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(""));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);

	modeLayer.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	progressBars.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	constLayer.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);
	frontSight.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(100);

	smaString.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(ShootMagAutoSTR));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	capString.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(CapSTR));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	warningString.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(WarningSTR));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	chassisModeString.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(chassisModeNormal));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
	aimModeString.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id, GET_STR(aimModeNoAuto));
	HAL_UART_Transmit(&huart2, UiBuffer, 62, 40);
	osDelay(100);
}

void UI_Refresh()
{
	UI_DataTypeDef *ui = UI_GetDataPtr();
	UI::Layer::getDelBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 19, 40);
	osDelay(100);
	UI_Init();
	osDelay(100);
}

uint16_t warn_count = 0;
void UI_Update()
{
	UI_DataTypeDef *ui = UI_GetDataPtr();
	Chassis_ControlTypeDef *chassis = Chassis_GetControlPtr();
	GimbalYaw_ControlTypeDef *gimbalyaw = GimbalYaw_GetControlPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	Cap_DataTypeDef *cap = Cap_GetDataPtr();

	ui->robot_id = boardcom->robot_id;
	ui->client_id = 0x100 + boardcom->robot_id;
	ui->rest_energy = cap->rest_energy;
	ui->pitch = -boardcom->pitch_angle;
	ui->mag_state = boardcom->magazine_state;
	ui->gyro_state = boardcom->chassis_mode == CHASSIS_GYRO;
	ui->shooter_state = boardcom->shooter_state;
	ui->auto_shoot_state = boardcom->auto_shoot_state;
	ui->cap_mode = boardcom->cap_speedup_flag;
	ui->flyslope_flag = boardcom->power_limit_mode;
	
	if (ui->shooter_state)
	{
		shooterRec.changeColor(UI::Color::Green);
	}
	else
	{
		shooterRec.changeColor(UI::Color::Pink);
	}
	
	if (ui->mag_state)
	{
		magRec.changeColor(UI::Color::Green);
	}
	else
	{
		magRec.changeColor(UI::Color::Pink);
	}
	
	if (ui->auto_shoot_state)
	{
		autoAimRec.changeColor(UI::Color::Green);
	}
	else
	{
		autoAimRec.changeColor(UI::Color::Pink);
	}
	if(ui->flyslope_flag == POWER_UNLIMIT)
	{
		capRec.changeColor(UI::Color::Purple);
	}
	else if (ui->flyslope_flag == POWER_LIMIT && ui->cap_mode == CAP_SPEEDUP)
	{
		capRec.changeColor(UI::Color::Green);
	}
	else if (ui->flyslope_flag == POWER_LIMIT && ui->cap_mode == CAP_NORMAL)
	{
		capRec.changeColor(UI::Color::Pink);
	}
	
	
	if (ui->homehurt)
	{
		homeWarningRec.changeColor(UI::Color::Pink);
	}
	else
	{
		homeWarningRec.changeColor(UI::Color::Green);
	}
	
	if (chassis->present_mode == CHASSIS_STOP)
	{
		ChangeModeStr(chassisModeString, GET_STR(chassisModeStop));
	}
	else if (chassis->present_mode == CHASSIS_NORMAL)
	{
		ChangeModeStr(chassisModeString, GET_STR(chassisModeNormal));
	}
	else if (chassis->present_mode == CHASSIS_GYRO)
	{
		ChangeModeStr(chassisModeString, GET_STR(chassisModeGyro));
	}
	
	if (gimbalyaw->present_mode == GimbalYaw_NO_AUTO)
	{
		ChangeModeStr(aimModeString, GET_STR(aimModeNoAuto));
	}
	else if (gimbalyaw->present_mode == GimbalYaw_ARMOR)
	{
		ChangeModeStr(aimModeString, GET_STR(aimModeArmor));
	}
	else if (gimbalyaw->present_mode == GimbalYaw_SMALL_ENERGY)
	{
		ChangeModeStr(aimModeString, GET_STR(aimModeSmallBuff));
	}
	else if (gimbalyaw->present_mode == GimbalYaw_BIG_ENERGY)
	{
		ChangeModeStr(aimModeString, GET_STR(aimModeBigBuff));
	}


	uint16_t l = eCapProgressBarLeft + 9 + static_cast<int16_t>((eCapProgressBarRight - eCapProgressBarLeft - 18) * ui->rest_energy / 100.0f);
	if (l > eCapProgressBarRight - 9)
	{
		l = eCapProgressBarRight - 9;
	}
	else if (l < eCapProgressBarLeft + 9)
	{
		l = eCapProgressBarLeft + 9;
	}
	eCapProgressBarCor.changeSecondPoint({l, eCapProgressBarBottom + 9});
	eCapProgressValue.changeValue(ui->rest_energy);
	pitchValue.changeValue(ui->pitch);

	modeLayer.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(50);
	progressBars.getUpdateBuffer(UiBuffer, ui->robot_id, ui->client_id);
	HAL_UART_Transmit(&huart2, UiBuffer, 122, 40);
	osDelay(50);
	
}

void HomeHurt_Detect()
{
    Referee_DataTypeDef *referee = Referee_GetDataPtr();
	BoardCom_DataTypeDef *boardcom = BoardCom_GetDataPtr();
	HP_DataTypeDef *hp = HP_GetDataPtr();
	UI_DataTypeDef *ui = UI_GetDataPtr();
	
	hp->blue_7_robot_HP_last = hp->blue_7_robot_HP;
	hp->blue_outpost_HP_last = hp->blue_outpost_HP;
	hp->blue_base_HP_last = hp->blue_base_HP;
	hp->red_7_robot_HP_last = hp->red_7_robot_HP;
	hp->red_outpost_HP_last = hp->red_outpost_HP;
	hp->red_base_HP_last = hp->red_base_HP;
	
	hp->blue_7_robot_HP = referee->blue_7_robot_HP;
	hp->blue_outpost_HP = referee->blue_outpost_HP;
	hp->blue_base_HP = referee->blue_base_HP;
	hp->red_7_robot_HP = referee->red_7_robot_HP;
	hp->red_outpost_HP = referee->red_outpost_HP;
	hp->red_base_HP = referee->red_base_HP;
	
	if (boardcom->team_color == COLOR_BLUE)
	{
		if (hp->blue_7_robot_HP < hp->blue_7_robot_HP_last || hp->blue_outpost_HP < hp->blue_outpost_HP_last || hp->blue_base_HP < hp->blue_base_HP_last)
		{
			ui->homehurt = 1;
		}
		else
		{
			ui->homehurt = 0;
		}
	}
	if (boardcom->team_color == COLOR_RED)
	{
		if (hp->red_7_robot_HP < hp->red_7_robot_HP_last || hp->red_outpost_HP < hp->red_outpost_HP_last || hp->red_base_HP < hp->red_base_HP_last)
		{
			ui->homehurt = 1;
		}
		else
		{
			ui->homehurt = 0;
		}
	}
}
