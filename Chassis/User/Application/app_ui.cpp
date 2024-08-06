#include "app_ui.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "cmsis_os.h"
#include "config_ctrl.h"
#include "lib_ui.h"
#include "protocol_referee.h"
#include "util_uart.h"

constexpr uint16_t CircleX = 960 + sight_bias;
constexpr uint16_t CircleY = 340;
constexpr uint16_t Radius = 60;
uint16_t directionline1FX = 0;
uint16_t directionline1FY = 0;
uint16_t directionline1SX = 0;
uint16_t directionline1SY = 0;
float direction_angle_sin = 0.0f;
float direction_angle_cos = 0.0f;
float direction_angle_err = 0.0f;
static float sin_table[181] = {0.000000, 0.008727, 0.017452, 0.026177, 0.034899, 0.043619, 0.052336, 0.061049, 0.069756, 0.078459, 0.087156, 0.095846, 0.104528, 0.113203, 0.121869, 0.130526, 0.139173, 0.147809, 0.156434, 0.165048, 0.173648, 0.182236, 0.190809, 0.199368, 0.207912, 0.216440, 0.224951, 0.233445, 0.241922, 0.250380, 0.258819, 0.267238, 0.275637, 0.284015, 0.292372, 0.300706, 0.309017, 0.317305, 0.325568, 0.333807, 0.342020, 0.350207, 0.358368, 0.366501, 0.374607, 0.382683, 0.390731, 0.398749, 0.406737, 0.414693, 0.422618, 0.430511, 0.438371, 0.446198, 0.453990, 0.461749, 0.469472, 0.477159, 0.484810, 0.492424, 0.500000, 0.507538, 0.515038, 0.522499, 0.529919, 0.537300, 0.544639, 0.551937, 0.559193, 0.566406, 0.573576, 0.580703, 0.587785, 0.594823, 0.601815, 0.608761, 0.615661, 0.622515, 0.629320, 0.636078, 0.642788, 0.649448, 0.656059, 0.662620, 0.669131, 0.675590, 0.681998, 0.688355, 0.694658, 0.700909, 0.707107, 0.713250, 0.719340, 0.725374, 0.731354, 0.737277, 0.743145, 0.748956, 0.754710, 0.760406, 0.766044, 0.771625, 0.777146, 0.782608, 0.788011, 0.793353, 0.798636, 0.803857, 0.809017, 0.814116, 0.819152, 0.824126, 0.829038, 0.833886, 0.838671, 0.843391, 0.848048, 0.852640, 0.857167, 0.861629, 0.866025, 0.870356, 0.874620, 0.878817, 0.882948, 0.887011, 0.891007, 0.894934, 0.898794, 0.902585, 0.906308, 0.909961, 0.913545, 0.917060, 0.920505, 0.923880, 0.927184, 0.930418, 0.933580, 0.936672, 0.939693, 0.942641, 0.945519, 0.948324, 0.951057, 0.953717, 0.956305, 0.958820, 0.961262, 0.963630, 0.965926, 0.968148, 0.970296, 0.972370, 0.974370, 0.976296, 0.978148, 0.979925, 0.981627, 0.983255, 0.984808, 0.986286, 0.987688, 0.989016, 0.990268, 0.991445, 0.992546, 0.993572, 0.994522, 0.995396, 0.996195, 0.996917, 0.997564, 0.998135, 0.998630, 0.999048, 0.999391, 0.999657, 0.999848, 0.999962, 1.000000};
static float cos_table[181] = {1.000000, 0.999962, 0.999848, 0.999657, 0.999391, 0.999048, 0.998630, 0.998135, 0.997564, 0.996917, 0.996195, 0.995396, 0.994522, 0.993572, 0.992546, 0.991445, 0.990268, 0.989016, 0.987688, 0.986286, 0.984808, 0.983255, 0.981627, 0.979925, 0.978148, 0.976296, 0.974370, 0.972370, 0.970296, 0.968148, 0.965926, 0.963630, 0.961262, 0.958820, 0.956305, 0.953717, 0.951057, 0.948324, 0.945519, 0.942641, 0.939693, 0.936672, 0.933580, 0.930418, 0.927184, 0.923880, 0.920505, 0.917060, 0.913545, 0.909961, 0.906308, 0.902585, 0.898794, 0.894934, 0.891007, 0.887011, 0.882948, 0.878817, 0.874620, 0.870356, 0.866025, 0.861629, 0.857167, 0.852640, 0.848048, 0.843391, 0.838671, 0.833886, 0.829038, 0.824126, 0.819152, 0.814116, 0.809017, 0.803857, 0.798636, 0.793353, 0.788011, 0.782608, 0.777146, 0.771625, 0.766044, 0.760406, 0.754710, 0.748956, 0.743145, 0.737277, 0.731354, 0.725374, 0.719340, 0.713250, 0.707107, 0.700909, 0.694658, 0.688355, 0.681998, 0.675590, 0.669131, 0.662620, 0.656059, 0.649448, 0.642788, 0.636078, 0.629320, 0.622515, 0.615661, 0.608761, 0.601815, 0.594823, 0.587785, 0.580703, 0.573576, 0.566406, 0.559193, 0.551937, 0.544639, 0.537300, 0.529919, 0.522499, 0.515038, 0.507538, 0.500000, 0.492424, 0.484810, 0.477159, 0.469472, 0.461749, 0.453990, 0.446198, 0.438371, 0.430511, 0.422618, 0.414693, 0.406737, 0.398749, 0.390731, 0.382683, 0.374607, 0.366501, 0.358368, 0.350207, 0.342020, 0.333807, 0.325568, 0.317305, 0.309017, 0.300706, 0.292372, 0.284015, 0.275637, 0.267238, 0.258819, 0.250380, 0.241922, 0.233445, 0.224951, 0.216440, 0.207912, 0.199368, 0.190809, 0.182236, 0.173648, 0.165048, 0.156434, 0.147809, 0.139173, 0.130526, 0.121869, 0.113203, 0.104528, 0.095846, 0.087156, 0.078459, 0.069756, 0.061049, 0.052336, 0.043619, 0.034899, 0.026177, 0.017452, 0.008727, 0.000000};
float get_table_sin(float angle)
{
	while (angle < 0.0f)
	{
		angle += 360.0f;
	}
	while (angle > 360.0f)
	{
		angle -= 360.0f;
	}
	if (angle <= 90.0f)
	{
		return sin_table[(int)(angle / 0.5)];
	}
	else if (angle <= 180.0f)
	{
		return sin_table[(int)((180.0f - angle) / 0.5)];
	}
	else if (angle <= 270.0f)
	{
		return -sin_table[(int)((angle - 180.0f) / 0.5)];
	}
	else
	{
		return -sin_table[(int)((360.0f - angle) / 0.5)];
	}
}
float get_table_cos(float angle)
{
	while (angle < 0.0f)
	{
		angle += 360.0f;
	}
	while (angle > 360.0f)
	{
		angle -= 360.0f;
	}
	if (angle <= 90.0f)
	{
		return cos_table[(int)(angle / 0.5)];
	}
	else if (angle <= 180.0f)
	{
		return -cos_table[(int)((180.0f - angle) / 0.5)];
	}
	else if (angle <= 270.0f)
	{
		return -cos_table[(int)((angle - 180.0f) / 0.5)];
	}
	else
	{
		return cos_table[(int)((360.0f - angle) / 0.5)];
	}
}
UI::Rectangle shooterRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220}, {1920 - 100 + 15, 1080 - 220 + 15}};

UI::Rectangle autoAimRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220 - 75}, {1920 - 100 + 15, 1080 - 220 + 15 - 75}};

UI::Rectangle capRec{
	UI::Color::Pink, 15, {1920 - 100, 1080 - 220 - 150}, {1920 - 100 + 15, 1080 - 220 + 15 - 150}};

UI::Rectangle homeWarningRec{
	UI::Color::Pink, 15, {300, 1080 - 250}, {300 + 15, 1080 - 250 + 15}};

constexpr uint16_t eCapProgressBarLeft = 360;
constexpr uint16_t eCapProgressBarTop = 851;
constexpr uint16_t eCapProgressBarRight = 645;
constexpr uint16_t eCapProgressBarBottom = 824;

UI ::Rectangle getTargetRec{
	UI::Color::Pink, 5, {960 - 140 + sight_bias, 540 - 140}, {960 + 140 + sight_bias, 540 + 140}};

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

UI::Float ChassisSpeedVz{
	UI::Color::Green, 2, {300, 540 + 160}, 20, 1.23456f};

UI::Float ChassisSpeedVx{
	UI::Color::Green, 2, {300, 540 + 120}, 20, 1.23456f};

UI::Float ChassisSpeedW{
	UI::Color::Green, 2, {300, 540 + 80}, 20, 1.23456f};

UI::Line frontSightMiddle{
	UI::Color::Black, 1, {960 + sight_bias, 690}, {960 + sight_bias, 390}};

UI::Line frontSightUp1{
	UI::Color::Black, 1, {885 + sight_bias, 690}, {1035 + sight_bias, 690}};

UI::Line frontSightUp2{
	UI::Color::Black, 1, {910 + sight_bias, 640}, {1010 + sight_bias, 640}};

UI::Line frontSightUp3{
	UI::Color::Black, 1, {935 + sight_bias, 590}, {985 + sight_bias, 590}};

UI::Line frontSightUp4{
	UI::Color::Black, 1, {945 + sight_bias, 665}, {975 + sight_bias, 665}};

UI::Line frontSightUp5{
	UI::Color::Black, 1, {945 + sight_bias, 615}, {975 + sight_bias, 615}};

UI::Line frontSightDown1{
	UI::Color::Black, 1, {885 + sight_bias, 390}, {1035 + sight_bias, 390}};

UI::Line frontSightDown2{
	UI::Color::Black, 1, {910 + sight_bias, 440}, {1010 + sight_bias, 440}};

UI::Line frontSightDown3{
	UI::Color::Black, 1, {935 + sight_bias, 490}, {985 + sight_bias, 490}};

UI::Line frontSightDown4{
	UI::Color::Black, 1, {945 + sight_bias, 415}, {975 + sight_bias, 415}};

UI::Line frontSightDown5{
	UI::Color::Black, 1, {945 + sight_bias, 465}, {975 + sight_bias, 465}};

UI::Circle DirectionCircle{
	UI::Color::Cyan, 3, {CircleX + sight_bias, CircleY}, Radius};

UI::Line DirectionLine1{
	UI::Color::Green, 4, {CircleX + sight_bias, CircleY}, {CircleX + sight_bias + 30, CircleY + 30}};

/* 实时更新的图层 */

UI::Layer modeLayer{
	"MD", 8, {&shooterRec, &autoAimRec, &capRec, &homeWarningRec, &getTargetRec, &DirectionLine1}};

UI::Layer progressBars{
	"PB", 8, {&eCapProgressBarCor, &eCapProgressValue, &pitchValue, &ChassisSpeedVz, &ChassisSpeedVx, &ChassisSpeedW}};

/* 只绘制，不修改的图层 */

UI::Layer constLayer{
	"CL", 8, {&eCapProgressBarBox, &frontSightUp4, &frontSightUp5, &frontSightDown4, &frontSightDown5, &DirectionCircle}};

UI::Layer frontSight{
	"FS",
	8,
	{&frontSightMiddle, &frontSightUp1, &frontSightUp2, &frontSightUp3, &frontSightDown1, &frontSightDown2,
	 &frontSightDown3}};

UI::String smaString{"SMA", 8, UI::Color::Cyan, 25, 3, {1920 - 300, 1080 - 200}};
#define ShootMagAutoSTR "SHOOTER:\n\n   AUTO:"

UI::String capString{"CAP", 8, UI::Color::Cyan, 25, 3, {1920 - 200, 1080 - 350}};
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
	ui->is_get_target = boardcom->is_get_target;
	ui->chassis_speed_vz = chassis->real_spd.vz;
	ui->chassis_speed_vx = chassis->real_spd.vx;
	ui->chassis_speed_w = chassis->real_spd.w;

	if (ui->shooter_state)
	{
		shooterRec.changeColor(UI::Color::Green);
	}
	else
	{
		shooterRec.changeColor(UI::Color::Pink);
	}
	if (ui->is_get_target)
	{
		getTargetRec.changeColor(UI::Color::Green);
	}
	else
	{
		getTargetRec.changeColor(UI::Color::Pink);
	}
	if (ui->auto_shoot_state)
	{
		autoAimRec.changeColor(UI::Color::Green);
	}
	else
	{
		autoAimRec.changeColor(UI::Color::Pink);
	}
	if (ui->flyslope_flag == POWER_UNLIMIT)
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

	if (ui->rest_energy < 30)
	{
		uint32_t _tick = HAL_GetTick();
		if (_tick % 1000 < 500)
		{
			eCapProgressBarCor.changeColor(UI::Color::Pink);
		}
		else
		{
			eCapProgressBarCor.changeColor(UI::Color::Green);
		}
	}
	else if (ui->rest_energy >= 30)
	{
		eCapProgressBarCor.changeColor(UI::Color::Green);
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
		chassisModeString.changeColor(UI::Color::Cyan);
	}
	else if (chassis->present_mode == CHASSIS_NORMAL)
	{
		chassisModeString.changeColor(UI::Color::Cyan);
		ChangeModeStr(chassisModeString, GET_STR(chassisModeNormal));
		chassisModeString.changeColor(UI::Color::Cyan);
	}
	else if (chassis->present_mode == CHASSIS_GYRO)
	{
		chassisModeString.changeColor(UI::Color::Orange);
		ChangeModeStr(chassisModeString, GET_STR(chassisModeGyro));
		chassisModeString.changeColor(UI::Color::Orange);
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

	direction_angle_err = Motor_GimbalYaw.encoder.limited_angle + 60.0f;
	direction_angle_sin = get_table_sin(direction_angle_err);
	direction_angle_cos = get_table_cos(direction_angle_err);

	directionline1FX = static_cast<int16_t>(CircleX - 60 * direction_angle_sin);
	directionline1FY = static_cast<int16_t>(CircleY - 60 * direction_angle_cos);
	directionline1SX = static_cast<int16_t>(CircleX + 60 * direction_angle_sin);
	directionline1SY = static_cast<int16_t>(CircleY + 60 * direction_angle_cos);

	DirectionLine1.changeFirstPoint({directionline1FX, directionline1FY});
	DirectionLine1.changeSecondPoint({directionline1SX, directionline1SY});

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

	ChassisSpeedVz.changeValue(ui->chassis_speed_vz);
	ChassisSpeedVx.changeValue(ui->chassis_speed_vx);
	ChassisSpeedW.changeValue(ui->chassis_speed_w);
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
