#include "robot_cmd.h"
#include "chassis.h"
#include "bsp_dwt.h"
#include "gimbal.h"
#include "stdio.h"
#include "shoot.h"
#include "bsp_temperature.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "dm_imu.h"

void robot_init(void)
{
    DWT_Init(168);
	IMUTempInit();
    RobotCMDInit();  
    GimbalInit();
    ChassisInit();
//	RefereeSend(tx_buffer,3);
    ShootInit();
    DM_IMU_Init();
}