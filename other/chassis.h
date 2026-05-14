#ifndef CHASSIS_H
#define CHASSIS_H
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "motor_def.h"
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit(void);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask(void);

#endif // CHASSIS_H


