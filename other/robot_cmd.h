#ifndef robot_CMD_H
#define robot_CMD_H

#define ROBOT_INFANTRY 3 

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit(void);

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask(void);

#endif // !GIMBAL_CMD_H
