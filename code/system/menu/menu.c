#include "menu.h"
#include "control.h"
#include "key.h"
#include "menu_input.h"
#include "motor.h"
#include "single_driver.h"
#include "system.h"
#include "test.h"
#include "guide.h"

//======================================================================================================================
typedef struct MENU_TABLE MENU_TABLE; // 鑿滃崟鎵ц
typedef struct MENU_PRMT MENU_PRMT;   // 鑿滃崟鍙傛暟
typedef enum MenuType MenuType;
typedef union Item Item;
typedef union MenuParam MenuParam;
typedef struct Site_t Site_t;
#define MenuNum(Table) sizeof(Table) / sizeof(Table[0])
#define Debug_Null NULL
void Menu_Null(void) {}

void Menu_Process(uint8 *menuName,
                  MENU_PRMT *prmt,
                  MENU_TABLE *table,
                  uint8 num);
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page);
void write_Flash(uint8 flashNum);
void read_Flash(uint8 flashNum);

MENU_TABLE Table_Null[] = {
    {(uint8 *)"", {.SubMenu = Table_Null}, Type_Null, {.ItemFunc = Menu_Null}},
};
//======================================================================================================================

uint8 g_exit_menu_flag = 0;
uint8 g_show_run_param_flag = 0;
uint32 *EEPROM_DATA_UINT[] = {
    (uint32 *)(&g_menu_manual_param.bottom_angle_parameter[0]),
    (uint32 *)(&g_menu_manual_param.bottom_angle_parameter[1]),
    (uint32 *)(&g_menu_manual_param.bottom_angle_parameter[2]),
    (uint32 *)(&g_menu_manual_param.bottom_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.bottom_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.bottom_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.bottom_angle_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.bottom_angle_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.bottom_angle_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.bottom_position_parameter[0]),
    (uint32 *)(&g_menu_manual_param.bottom_position_parameter[1]),
    (uint32 *)(&g_menu_manual_param.bottom_position_parameter[2]),
    (uint32 *)(&g_menu_manual_param.side_angle_parameter[0]),
    (uint32 *)(&g_menu_manual_param.side_angle_parameter[1]),
    (uint32 *)(&g_menu_manual_param.side_angle_parameter[2]),
    (uint32 *)(&g_menu_manual_param.side_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.side_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.side_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.side_angle_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.side_angle_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.side_angle_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.turn_angle_parameter[0]),
    (uint32 *)(&g_menu_manual_param.turn_angle_parameter[1]),
    (uint32 *)(&g_menu_manual_param.turn_angle_parameter[2]),
    (uint32 *)(&g_menu_manual_param.turn_angle_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.turn_angle_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.turn_angle_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.turn_error_parameter[0]),
    (uint32 *)(&g_menu_manual_param.turn_error_parameter[1]),
    (uint32 *)(&g_menu_manual_param.turn_error_parameter[2]),
    (uint32 *)(&g_menu_manual_param.turn_velocity_parameter[0]),
    (uint32 *)(&g_menu_manual_param.turn_velocity_parameter[1]),
    (uint32 *)(&g_menu_manual_param.turn_velocity_parameter[2]),
    (uint32 *)(&g_menu_manual_param.FrontControlTimeParameter[0]),
    (uint32 *)(&g_menu_manual_param.FrontControlTimeParameter[1]),
    (uint32 *)(&g_menu_manual_param.FrontControlTimeParameter[2]),
    (uint32 *)(&g_menu_manual_param.SideControlTimeParameter[0]),
    (uint32 *)(&g_menu_manual_param.SideControlTimeParameter[1]),
    (uint32 *)(&g_menu_manual_param.SideControlTimeParameter[2]),
    (uint32 *)(&g_menu_manual_param.TurnControlTimeParameter[0]),
    (uint32 *)(&g_menu_manual_param.TurnControlTimeParameter[1]),
    (uint32 *)(&g_menu_manual_param.TurnControlTimeParameter[2]),
    (uint32 *)(&g_menu_manual_param.TurnControlTimeParameter[3]),
    (uint32 *)(&g_control_shutdown_flag),
    (uint32 *)(&g_control_bottom_flag),
    (uint32 *)(&g_control_side_flag),
    (uint32 *)(&g_control_output_sav_flag),
    (uint32 *)(&g_control_output_sv_flag),
    (uint32 *)(&g_control_output_sa_flag),
    (uint32 *)(&g_control_output_fa_flag),
    (uint32 *)(&g_control_output_fv_flag),
    (uint32 *)(&g_control_output_fav_flag),
    (uint32 *)(&g_show_run_param_flag),
    (uint32 *)(&g_menu_manual_param.angle_limit),
};

int32 *EEPROM_DATA_INT[] = {
    (int32 *)(&g_menu_manual_param.mechanicalYawAngle),
    (int32 *)(&g_menu_manual_param.mechanicalPitchAngle),
    (int32 *)(&g_menu_manual_param.mechanicalRollAngle),
    (int32 *)(&g_menu_manual_param.bottom_velocity),
    (int32 *)(&g_menu_manual_param.turn_target),
    (int32 *)(&g_menu_manual_param.side_internal_diff),
    (int32 *)(&g_menu_manual_param.buckling_front_coefficient),
    (int32 *)(&g_menu_manual_param.buckling_side_coefficient),
};

MENU_PRMT PID_Prmt;

MENU_TABLE PID_Table[] = {
    {(uint8 *)"FAV_kp",
     {.UINT32 =
          (uint32 *)&g_menu_manual_param.bottom_angle_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FAV_ki",
     {.UINT32 =
          (uint32 *)&g_menu_manual_param.bottom_angle_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FAV_kd",
     {.UINT32 =
          (uint32 *)&g_menu_manual_param.bottom_angle_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FA_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_angle_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FA_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_angle_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FA_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_angle_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FV_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FV_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"FV_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.bottom_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SAV_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SAV_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SAV_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SA_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SA_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SA_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_angle_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SV_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SV_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"SV_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.side_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},

    {(uint8 *)"TAV_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TAV_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TAV_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TE_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_error_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TE_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_error_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TE_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_error_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TV_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_velocity_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TV_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_velocity_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TV_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_velocity_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TA_kp",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TA_ki",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TA_kd",
     {.UINT32 = (uint32 *)&g_menu_manual_param.turn_angle_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"BP_kp",
     {.INT32 = (uint32 *)&g_menu_manual_param.bottom_position_parameter[0]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"BP_ki",
     {.INT32 = (uint32 *)&g_menu_manual_param.bottom_position_parameter[1]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"BP_kd",
     {.INT32 = (uint32 *)&g_menu_manual_param.bottom_position_parameter[2]},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
};

MENU_TABLE Pid_TimeMenuTable[] = {
    {(uint8 *)"FAV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.FrontControlTimeParameter[0]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"FA_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.FrontControlTimeParameter[1]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"FV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.FrontControlTimeParameter[2]}, Param_Uint, {.ItemFunc = Menu_Null}},

    {(uint8 *)"SAV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.SideControlTimeParameter[0]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"SA_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.SideControlTimeParameter[1]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"SV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.SideControlTimeParameter[2]}, Param_Uint, {.ItemFunc = Menu_Null}},

    {(uint8 *)"TAV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.TurnControlTimeParameter[0]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"TE_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.TurnControlTimeParameter[1]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"TV_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.TurnControlTimeParameter[2]}, Param_Uint, {.ItemFunc = Menu_Null}},
    {(uint8 *)"TA_Time", {.UINT32 = (uint32 *)&g_menu_manual_param.TurnControlTimeParameter[3]}, Param_Uint, {.ItemFunc = Menu_Null}},
};

MENU_TABLE Buckling[] = {
    {(uint8 *)"bucking_turn",
     {.INT32 = (int32 *)&g_menu_manual_param.buckling_side_coefficient},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"bucking_front",
     {.INT32 = (int32 *)&g_menu_manual_param.buckling_front_coefficient},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    // {(uint8*)"turnCurvature",
    //  {.INT32 = (int32*)&turnCurvatureTest},
    //  Param_Int,
    //  {.ItemFunc = Menu_Null}},
    // {(uint8 *)"T_coeff",
    //  {.UINT32 = (uint32 *)&g_menu_manual_param.turnGainCoefficient},
    //  Param_Uint,
    //  {.ItemFunc = Menu_Null}},
    // {(uint8 *)"FFF_coeff",
    //  {.UINT32 = (uint32 *)&g_menu_manual_param.bucklingFrontCoefficientV},
    //  Param_Uint,
    //  {.ItemFunc = Menu_Null}},
    // {(uint8 *)"BFT_coeff",
    //  {.UINT32 = (uint32 *)&g_menu_manual_param.bucklingFrontCoefficientT},
    //  Param_Uint,
    //  {.ItemFunc = Menu_Null}},
};

MENU_TABLE Calibration_MenuTable[] = {
    {(uint8 *)"pitch",
     {.INT32 = (int32 *)&g_menu_manual_param.mechanicalPitchAngle},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"roll",
     {.INT32 = (int32 *)&g_menu_manual_param.mechanicalRollAngle},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"yaw",
     {.INT32 = (int32 *)&g_menu_manual_param.mechanicalYawAngle},
     Param_Int,
     {.ItemFunc = Menu_Null}},
};

MENU_TABLE Test_MenuTable[] = {
    {(uint8 *)"BOTTOM",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_bottom_motor}},
    {(uint8 *)"SIDE",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_side_motor}},
    {(uint8 *)"Attitude",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_attitude}},
    {(uint8 *)"IMU",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_imu}},
    {(uint8 *)"Noise",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_noise}},
    {(uint8 *)"SDeadzone",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_side_deadzone}},
    {(uint8 *)"BDeadzone",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_bottom_deadzone}},
    {(uint8 *)"SetBpwm",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_bottom_pwm}},
    {(uint8 *)"Grey",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_double_camera}},
    {(uint8 *)"Binary",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_image}},
    {(uint8 *)"Assisant",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_send_img}},
    {(uint8 *)"Key",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_key}},
    {(uint8 *)"SD_Card",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_sd_card}},
    {(uint8 *)"Receiver",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_receiver}},
    {(uint8 *)"Img_Shoot",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_img_shoot}},
    {(uint8 *)"DrawLine",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_line}},
    {(uint8 *)"YawInteg",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_yaw_integral}},
    {(uint8 *)"EncoderToVel",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_encoder_to_velocity}},
    {(uint8 *)"Switch",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_switch}},
    {(uint8 *)"Diode",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_diode}},
    {(uint8 *)"DualCam",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_dual_camera}},
    {(uint8 *)"CpuFreq",
     {.SubMenu = Table_Null},
     Functions,
     {.ItemFunc = test_cpu_freq}},
};

MENU_TABLE Utils_MenuTable[] = {
    {(uint8 *)"FV",
     {.INT32 = (int32 *)&g_menu_manual_param.bottom_velocity},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"TURN_TARGET",
     {.INT32 = (int32 *)&g_menu_manual_param.turn_target},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"diff",
     {.INT32 = (int32 *)&g_menu_manual_param.side_internal_diff},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"angle limit",
     {.UINT32 = (uint32 *)&g_menu_manual_param.angle_limit},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
};

MENU_TABLE Setting_MenuTable[] = {
    {(uint8 *)"sav_output",
     {.INT32 = (int32 *)&g_control_output_sav_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"sa_output",
     {.INT32 = (int32 *)&g_control_output_sa_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"sv_output",
     {.INT32 = (int32 *)&g_control_output_sv_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"fa_output",
     {.INT32 = (int32 *)&g_control_output_fa_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"fv_output",
     {.INT32 = (int32 *)&g_control_output_fv_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"fav_output",
     {.INT32 = (int32 *)&g_control_output_fav_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
};

MENU_TABLE MainMenu_Table[] = {
    {(uint8 *)"shutdown",
     {.INT32 = (int32 *)&g_control_shutdown_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"bottom",
     {.INT32 = (int32 *)&g_control_bottom_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"side",
     {.INT32 = (int32 *)&g_control_side_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"turn",
     {.INT32 = (int32 *)&g_control_turn_flag},
     Param_Int,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"LCD",
     {.INT32 = (int32 *)&g_show_run_param_flag},
     Param_Uint,
     {.ItemFunc = Menu_Null}},
    {(uint8 *)"1.PID_Param",
     {.SubMenu = PID_Table},
     Sub_Menus,
     {.SubMenuNum = MenuNum(PID_Table)}},
    {(uint8 *)"2.Utils",
     {.SubMenu = Utils_MenuTable},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Utils_MenuTable)}},
    {(uint8 *)"3.PID_TimePara",
     {.SubMenu = Pid_TimeMenuTable},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Pid_TimeMenuTable)}},
    {(uint8 *)"4.Buckling",
     {.SubMenu = Buckling},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Buckling)}},
    {(uint8 *)"5.Calibra",
     {.SubMenu = Calibration_MenuTable},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Calibration_MenuTable)}},
    {(uint8 *)"6.Test",
     {.SubMenu = Test_MenuTable},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Test_MenuTable)}},
    {(uint8 *)"7.Setting",
     {.SubMenu = Setting_MenuTable},
     Sub_Menus,
     {.SubMenuNum = MenuNum(Setting_MenuTable)}},
};

/******************************************************************************
 * FunctionName   : MainMenu_Set()
 * Description    : 甯歌璁剧疆
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void MainMenu_Set()
{
    // ExitMenu_flag = 0;
    lcd_clear();
    MENU_PRMT MainMenu_Prmt;
    uint8 menuNum =
        sizeof(MainMenu_Table) / sizeof(MainMenu_Table[0]); // 鑿滃崟椤规暟
    Menu_Process((uint8 *)" -=    Setting   =- ", &MainMenu_Prmt, MainMenu_Table,
                 menuNum);
    Write_EEPROM(); // 灏嗘暟鎹啓鍏EPROM淇濆瓨
    lcd_clear();
}
//================================================================================================================
// 鑿滃崟鍩烘湰鍑芥暟
// 涓嶇敤绠¤繖涓儴鍒� 鍩烘湰涓嶇敤鏀�
/******************************************************************************
 * FunctionName   : Menu_PrmtInit()
 * Description    : 鍒濆鍖栬彍鍗曞弬鏁�
 * EntryParameter : prmt - 鑿滃崟鍙傛暟, num - 姣忛〉鏄剧ず椤规暟, page - 鏈�澶ф樉绀洪〉鏁�
 * ReturnValue    : None
 *******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
    prmt->ExitMark = 0;   // 娓呴櫎閫�鍑鸿彍鍗曟爣蹇�
    prmt->Cursor = 0;     // 鍏夋爣娓呴浂
    prmt->PageNo = 0;     // 椤垫竻闆�
    prmt->Index = 0;      // 绱㈠紩娓呴浂
    prmt->DispNum = num;  // 椤垫渶澶氭樉绀洪」鐩暟
    prmt->MaxPage = page; // 鏈�澶氶〉鏁�
}
/******************************************************************************
 * FunctionName   : Menu_Move()
 * Description    : 鑿滃崟绉诲姩
 * EntryParameter : prmt - 鑿滃崟鍙傛暟, key - 鎸夐敭鍊�
 * ReturnValue    : 鏈夌‘璁よ繑鍥�0锛屽惁鍒欒繑鍥�1
 ******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key)
{
    uint8 rValue = 1;
    switch (key)
    {
    case KEY_U: // 鍚戜笂
    {
        if (prmt->Cursor != 0) // 鍏夋爣涓嶅湪椤剁
        {
            prmt->Cursor--; // 鍏夋爣涓婄Щ
        }
        else // 鍏夋爣鍦ㄩ《绔�
        {
            if (prmt->PageNo != 0) // 椤甸潰娌℃湁鍒版渶灏�
            {
                prmt->PageNo--; // 鍚戜笂缈�
            }
            else
            {
                prmt->Cursor = prmt->DispNum - 1; // 鍏夋爣鍒板簳
                prmt->PageNo = prmt->MaxPage - 1; // 鏈�鍚庨〉
            }
        }
        break;
    }

    case KEY_D: // 鍚戜笅
    {
        if (prmt->Cursor < prmt->DispNum - 1) // 鍏夋爣娌℃湁鍒板簳锛岀Щ鍔ㄥ厜鏍�
        {
            prmt->Cursor++; // 鍏夋爣鍚戜笅绉诲姩
        }
        else // 鍏夋爣鍒板簳
        {
            if (prmt->PageNo < prmt->MaxPage - 1) // 椤甸潰娌℃湁鍒板簳锛岄〉闈㈢Щ鍔�
            {
                prmt->PageNo++; // 涓嬬炕涓�椤�
            }
            else // 椤甸潰鍜屽厜鏍囬兘鍒板簳锛岃繑鍥炲紑濮嬮〉
            {
                prmt->Cursor = 0;
                prmt->PageNo = 0;
            }
        }
        break;
    }
    case KEY_B: // 纭
    {
        prmt->Index = prmt->Cursor + prmt->PageNo; // 璁＄畻鎵ц椤圭殑绱㈠紩
        rValue = 0;

        break;
    }
    case KEY_L: // 宸﹂敭杩斿洖涓婄骇鑿滃崟
    {
        prmt->ExitMark = 1;

        break;
    }
    case KEY_R: // 鍙抽敭璺冲埌搴曢儴
    {
        prmt->Cursor = prmt->DispNum - 1; // 鍏夋爣鍒板簳
        prmt->PageNo = prmt->MaxPage - 1; // 鏈�鍚庨〉
        break;
    }
    default:
        break;
    }
    return rValue; // 杩斿洖鎵ц绱㈠紩
}
/******************************************************************************
 * FunctionName   : KeySan()
 * Description    : 鎸夐敭鑾峰彇
 * EntryParameter : None
 * ReturnValue    : 鎸夐敭鍊�
 *******************************************************************************/
KEY_e KeySan(void)
{
    KEY_e key_temp = KEY_NONE;
    while (keymsg.status == KEY_UP)
    {
        // 绛夊緟鎸夐敭琚寜涓�
        system_delay_ms(10); // 閬垮厤杩囧害鍗犵敤CPU
    }

    key_temp = keymsg.key; // 淇濆瓨褰撳墠鎸夐敭鍊�
    key_clear_msg();       // 娓呴櫎鎸夐敭鐘舵��
    return key_temp;       // 杩斿洖鎸夐敭鍊�
}
/******************************************************************************
 * FunctionName   : SubNameCat()
 * Description    : 鐢熸垚瀛愯彍鍗曟爣棰�
 * EntryParameter : None
 * ReturnValue    : void
 *******************************************************************************/
void SubNameCat(uint8 *SubMenuName, uint8 *TableMenuName)
{
    const uint8 SubTitlePrefix[] = "-=";
    const uint8 SubTitleSuffix[] = "=-";
    for (uint8 i = 0; i < 20; i++)
    { // initiate the string
        SubMenuName[i] = '\0';
    }
    for (uint8 i = 0; i < strlen((char *)TableMenuName);
         i++)
    { // find the first space and cut the string
        if (TableMenuName[i] == ' ')
        {
            TableMenuName[i] = '\0';
            break;
        }
    }
    if (strlen((char *)TableMenuName) >
        14)
    { // if the name is too long, we need to cut it
        TableMenuName[14] = '\0';
        strcat((char *)SubMenuName, (char *)SubTitlePrefix);
        strcat((char *)SubMenuName, (char *)TableMenuName);
        strcat((char *)SubMenuName, (char *)SubTitleSuffix);
    }
    else
    {
        strcat((char *)SubMenuName, (char *)SubTitlePrefix);
        uint8 spaceNum = (14 - strlen((char *)TableMenuName)) / 2;
        for (uint8 i = 0; i < spaceNum; i++)
        { // fill the space
            strcat((char *)SubMenuName, " ");
        }
        strcat((char *)SubMenuName, (char *)TableMenuName);
        for (uint8 i = 0; i < 14 - spaceNum - strlen((char *)TableMenuName);
             i++)
        { // fill the space
            strcat((char *)SubMenuName, " ");
        }
        strcat((char *)SubMenuName, (char *)SubTitleSuffix);
    }
    // const uint8 SubTitle1[] = " -=";
    // const uint8 SubTitle2[] = "=- ";
    // for (uint8 i = 0;i < 20;i++){
    //     SubMenuName[i] = '\0';
    // }
    // uint8 TableBody[20];
    // uint8 NameLenth = (uint8)strlen((char*)TableMenuName);
    // for (uint8 i = 0;i < NameLenth;i ++){
    //     TableBody[i] = TableMenuName[i];
    //     if(TableMenuName[i] == ' ') {
    //         TableBody[i] = '\0';
    //         NameLenth = i;
    //     }
    // }
    // if (NameLenth > 14) NameLenth = 14;
    // strcat((char*)SubMenuName,(char*)SubTitle1);
    // for (uint8 i = 0;i < (14-NameLenth)/2         ;i ++)
    // strcat((char*)SubMenuName," "); //濉厖绌烘牸
    // strcat((char*)SubMenuName,(char*)TableBody);
    // for (uint8 i = 0;i < 7-NameLenth + NameLenth/2;i ++)
    // strcat((char*)SubMenuName," "); //濉厖绌烘牸
    // strcat((char*)SubMenuName,(char*)SubTitle2);
}
/******************************************************************************
 * FunctionName   : adjustParam()
 * Description    : 璋冩暣鍙傛暟
 * EntryParameter : None
 * ReturnValue    : void
 *******************************************************************************/
void adjustParam(Site_t site, MENU_TABLE *table)
{
    KEY_e key;
    do
    {
        key = KeySan(); // 浣跨敤淇敼鍚庣殑KeySan鍑芥暟锛屼細鑷姩娓呴櫎鎸夐敭鐘舵��
        MenuParam param;
        if (table->MenuType == Param_Uint)
            param.UINT32 = table->MenuParams.UINT32;
        else
            param.INT32 = table->MenuParams.INT32;
        switch (key)
        {
        case KEY_U:
            if (table->MenuType == Param_Uint)
                (*param.UINT32)++;
            else if (table->MenuType == Param_Int)
                (*param.INT32)++;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 < EnumNameNum - 1)
                    (*param.INT32)++;
            }
            break;

        case KEY_D:
            if (table->MenuType == Param_Uint)
                (*param.UINT32)--;
            else if (table->MenuType == Param_Int)
                (*param.INT32)--;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 > 0)
                    (*param.INT32)--;
            }
            break;

        case KEY_L:
            if (table->MenuType == Param_Uint)
                (*param.UINT32) -= 10;
            else if (table->MenuType == Param_Int)
                (*param.INT32) -= 10;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 > 5)
                    (*param.INT32) -= 5;
            }
            break;

        case KEY_R:
            if (table->MenuType == Param_Uint)
                (*param.UINT32) += 10;
            else if (table->MenuType == Param_Int)
                (*param.INT32) += 10;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 < EnumNameNum - 5)
                    (*param.INT32) += 5;
            }
            break;

        default:
            break;
        }
        if (table->MenuType == Param_Uint)
        {
            lcd_show_uint_color(site.x, site.y, *param.UINT32, 6,
                                DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
            if (table->ItemHook.ItemFunc != Menu_Null)
                table->ItemHook.ItemFunc();
        }
        else if (table->MenuType == Param_Int)
        {
            lcd_show_int_color(site.x, site.y, *param.INT32, 6,
                               DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
            if (table->ItemHook.ItemFunc != Menu_Null)
                table->ItemHook.ItemFunc();
        }
        else if (table->MenuType == Enumerate)
        {
            lcd_show_string(site.x, site.y,
                            (int8 *)(table->ItemHook.EnumName +
                                     (*param.INT32) * (EnumNameLenth + 1)));
        }
    } while (key != KEY_B);
}

/******************************************************************************
 * FunctionName   : Menu_Display()
 * Description    : 鏄剧ず鑿滃崟椤�
 * EntryParameter : page - 鏄剧ず椤碉紝dispNum - 姣忎竴椤电殑鏄剧ず椤癸紝cursor - 鍏夋爣浣嶇疆
 * ReturnValue    : None
 *******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable,
                  uint8 pageNo,
                  uint8 dispNum,
                  uint8 cursor)
{
    uint8 i;
    Site_t site;
    for (i = 0; i < dispNum; i++)
    {
        site.x = 0;
        site.y = i + 1;
        if (cursor == i)
            /* 鍙嶇櫧鏄剧ず褰撳墠鍏夋爣閫変腑鑿滃崟椤� */
            lcd_show_string_color((uint16)site.x, (uint16)site.y,
                                  (const int8 *)menuTable[pageNo + i].MenuName,
                                  DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
        else
            /* 姝ｅ父鏄剧ず鍏朵綑鑿滃崟椤� */
            lcd_show_string_color((uint16)site.x, (uint16)site.y,
                                  (const int8 *)menuTable[pageNo + i].MenuName,
                                  DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
        /* 鑻ユ鑿滃崟椤规湁闇�瑕佽皟鐨勫弬鏁帮紝鍒欐樉绀鸿鍙傛暟 */
        if (menuTable[pageNo + i].MenuType == Param_Uint ||
            menuTable[pageNo + i].MenuType == Param_Int ||
            menuTable[pageNo + i].MenuType == Enumerate)
        {
            site.x = ALIGN_DIST;
            if (menuTable[pageNo + i].MenuType == Param_Uint)
            {
                uint32 num_t = (*(menuTable[pageNo + i].MenuParams.UINT32));
                lcd_show_uint(site.x, site.y, num_t, 6);
            }
            else if (menuTable[pageNo + i].MenuType == Param_Int)
            {
                int32 num_t = (*(menuTable[pageNo + i].MenuParams.INT32));
                lcd_show_int(site.x, site.y, num_t, 6);
            }
            else if (menuTable[pageNo + i].MenuType == Enumerate)
            {
                uint32 num_t = (*(menuTable[pageNo + i].MenuParams.UINT32));
                lcd_show_string(
                    site.x, site.y,
                    (int8 *)(menuTable[pageNo + i].ItemHook.EnumName +
                             num_t * (EnumNameLenth + 1)));
            }
        }
    }
}

/******************************************************************************
 * FunctionName   : Menu_Process()
 * Description    : 澶勭悊鑿滃崟椤�
 * EntryParameter : menuName - 鑿滃崟鍚嶇О锛宲rmt - 鑿滃崟鍙傛暟锛宼able - 鑿滃崟琛ㄩ」, num
 *- 鑿滃崟椤规暟 ReturnValue    : None Describe
 *: 1.杩涘叆瀛愯彍鍗� 2.璋冭妭鍙傛暟 3.璋冭妭鍙傛暟骞舵墽琛� 4.鎵ц鍑芥暟
 ******************************************************************************/
void Menu_Process(uint8 *menuName,
                  MENU_PRMT *prmt,
                  MENU_TABLE *table,
                  uint8 num)
{
    KEY_e key;
    Site_t site;
    uint8 page; // 鏄剧ず鑿滃崟闇�瑕佺殑椤垫暟
    if (num - PAGE_DISP_NUM <= 0)
        page = 1;
    else
    {
        page = num - PAGE_DISP_NUM + 1;
        num = PAGE_DISP_NUM;
    }
    // 鏄剧ず椤规暟鍜岄〉鏁拌缃�
    Menu_PrmtInit(prmt, num, page);
    do
    {
        lcd_clear();
        lcd_show_string(0, 0, (const int8 *)menuName); // 鏄剧ず鑿滃崟鏍囬
        // 鏄剧ず鑿滃崟椤�
        Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
        key = KeySan(); // 鑾峰彇鎸夐敭

        if (Menu_Move(prmt, key) == 0) // 鑿滃崟绉诲姩 鎸変笅纭閿�
        {
            // 鍒ゆ柇姝よ彍鍗曢」鏈夋棤闇�瑕佽皟鑺傜殑鍙傛暟 鏈夊垯杩涘叆鍙傛暟璋冭妭
            // 鍦ㄥ弬鏁拌皟鑺傞噷鐪嬫湁鏃犲嚱鏁板悓鏃惰繍琛�
            if (table[prmt->Index].MenuType == Param_Uint ||
                table[prmt->Index].MenuType == Param_Int ||
                table[prmt->Index].MenuType == Enumerate)
            {
                site.x = ALIGN_DIST;
                site.y = 1 + prmt->Cursor;
                if (table[prmt->Index].MenuType == Param_Uint)
                    lcd_show_uint_color(
                        site.x, site.y, *(table[prmt->Index].MenuParams.UINT32),
                        6, DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
                else if (table[prmt->Index].MenuType == Param_Int)
                    lcd_show_int_color(
                        site.x, site.y, *(table[prmt->Index].MenuParams.INT32),
                        6, DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
                else if (table[prmt->Index].MenuType == Enumerate)
                    lcd_show_string(
                        site.x, site.y,
                        (int8 *)(table[prmt->Index].ItemHook.EnumName +
                                 (*(table[prmt->Index].MenuParams.INT32)) *
                                     EnumNameLenth));
                // 鍦ㄥ弬鏁拌皟鑺傞噷鐪嬫湁鏃犲嚱鏁板悓鏃惰繍琛�  鍙互鍚屾椂鎵ц
                // 鏂逛究鑸垫満璋冭瘯锛岀數鏈鸿皟璇� 杩欎釜鍦ㄤ笂闈㈢殑璋冭妭鍙傛暟鍑芥暟閲屽凡缁忔墽琛岃繃
                adjustParam(site, &table[prmt->Index]);
            }
            // 涓嶆槸鍙傛暟璋冭妭鐨勮瘽灏辨墽琛岃彍鍗曞嚱鏁�
            else if (table[prmt->Index].MenuType == Functions)
            {
                table[prmt->Index].ItemHook.ItemFunc(); // 鎵ц鐩稿簲椤�
            }
            // 娌℃湁鍙傛暟璋冭妭鍜屽嚱鏁版墽琛岀殑璇� 灏辨槸瀛愯彍鍗�
            else if (table[prmt->Index].MenuType == Sub_Menus)
            {
                // 纭畾鏈夊瓙鑿滃崟
                //                if (table[prmt->Index].MenuParams.SubMenu !=
                //                Table_Null){
                lcd_clear();
                MENU_PRMT Submenu_Prmt;
                uint8 SubMenuName[20];
                SubNameCat(SubMenuName, table[prmt->Index].MenuName);
                Menu_Process(SubMenuName, &Submenu_Prmt,
                             table[prmt->Index].MenuParams.SubMenu,
                             table[prmt->Index].ItemHook.SubMenuNum);
                //                }
            }
        }
        // } while (prmt->ExitMark == 0 && ExitMenu_flag == 0);
    } while (prmt->ExitMark == 0);
    lcd_clear();
}

void write_Flash(uint8 flashNum)
{
    /* 涓�鍏辨湁96KB 96KB鍒嗕负浜�12椤� 姣忛〉鍙互瀛�1024涓猽int32绫诲瀷鐨勬暟鎹� 浠呬娇鐢ㄧ0椤� */
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_erase_page(0, (uint32)flashNum);

    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
        flash_union_buffer[i].uint32_type = (uint32)*EEPROM_DATA_UINT[i];
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
        flash_union_buffer[i + Flash_Save_intNum].int32_type =
            (int32)*EEPROM_DATA_INT[i];
    flash_write_page_from_buffer(0, (uint32)flashNum);

    flash_buffer_clear(); // 娓呴櫎缂撳瓨
    lcd_show_string(0, 0, "WRITE IS OK!");
}

void read_Flash(uint8 flashNum)
{
    /* 姣忛〉鍙互瀛�1024涓猽int32绫诲瀷鐨勬暟鎹� */
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_buffer_clear();                   // 娓呴櫎缂撳瓨
    flash_read_page_to_buffer(0, flashNum); // 灏嗘暟鎹粠缂撳瓨鍖鸿鍑烘潵
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA_UINT[i] = temp_vaule;
    }
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
    {
        int32 temp_vaule =
            flash_union_buffer[Flash_Save_uintNum + i].int32_type;
        *EEPROM_DATA_INT[i] = temp_vaule;
    }
    flash_buffer_clear(); // 娓呴櫎缂撳瓨
    lcd_show_string(0, 0, "READ IS OK!");
}

void Read_EEPROM()
{
    lcd_clear();
    lcd_show_string(0, 0, "READ EEPROM");
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_buffer_clear();             // 娓呴櫎缂撳瓨
    flash_read_page_to_buffer(0, 11); // 灏嗘暟鎹粠缂撳瓨鍖鸿鍑烘潵
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA_UINT[i] = temp_vaule;
    }
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
    {
        int32 temp_vaule =
            flash_union_buffer[Flash_Save_uintNum + i].int32_type;
        *EEPROM_DATA_INT[i] = temp_vaule;
    }
    flash_buffer_clear(); // 娓呴櫎缂撳瓨
    lcd_clear();
    lcd_show_string(0, 0, "READ SUCCESS");
}

void Write_EEPROM()
{
    lcd_clear();
    lcd_show_string(0, 0, "WRITE EEPROM");
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_erase_page(0, 11);
    flash_buffer_clear(); // 娓呴櫎缂撳瓨
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
        flash_union_buffer[i].uint32_type = (uint32)*EEPROM_DATA_UINT[i];
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
        flash_union_buffer[i + Flash_Save_uintNum].int32_type =
            (int32)*EEPROM_DATA_INT[i];
    flash_write_page_from_buffer(0, 11);
    flash_buffer_clear(); // 娓呴櫎缂撳瓨
    lcd_clear();
    lcd_show_string(0, 0, "WRITE SUCCESS");
}
