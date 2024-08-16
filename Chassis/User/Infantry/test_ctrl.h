/*
 * @Project: Infantry Code
 * @Author: GDDG08
 * @Date: 2024-07-18 18:26:59
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-08-16 17:55:07
 */
// 条件编译控制台，实现开关式的操作
#define OLD_WHEAT 1
#define SWING_DANCE 2
#define BIG_TITAN 3
#define STAR_DUST 4
#define HUGE_TITAN 5
#define WHITE_MISTRESS 6
#define ROBOT_ID STAR_DUST // 选车（这份代码只包含分区赛全向（4号）和国赛全向（6号））

#define NO_SYS_IDENT 1
#define SYS_IDENT 0
#define IF_SYS_IDENT NO_SYS_IDENT // 是否进行系统辨识

#define FOLLOW 1
#define NO_FOLLOW 0
#define IF_FOLLOW FOLLOW // 是否开启随动

#define YAW_REMOTE 1
#define YAW_STEP 0
#define YAW_MOVE YAW_REMOTE // yaw轴调参设置，选择输入方式为遥控器或手动阶跃
