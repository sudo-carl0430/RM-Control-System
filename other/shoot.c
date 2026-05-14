#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

static float change_speed=0; //设置缓启动

static DJIMotorInstance *friction_l; // 左摩擦轮
static DJIMotorInstance *friction_r; // 右摩擦轮
static DJIMotorInstance *loader;     // 拨盘电机

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自gimbal_cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; 

static float hibernate_time = 0, dead_time = 0;
static float load_angle_set = 0;
float bullet_speed = 0; 
static float last_bullet_speed = 0;
static int fric_rate = 40000;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1,//20
                .Ki = 0,//1
                .Kd = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve = PID_Integral_Limit| PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 1.5,//0.7
                .Ki = 0,//0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP ,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,

        },
        .motor_type = M3508};

#ifdef ROBOT_MECANUM    //麦轮摩擦轮方向，左为2，右为1
        friction_config.can_init_config.tx_id = 2,
        friction_l = DJIMotorInit(&friction_config);

        friction_config.can_init_config.tx_id = 1; // 右摩擦轮,改txid和方向就行
        friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
        friction_r = DJIMotorInit(&friction_config); 
#endif

#ifdef ROBOT_OMNIDIRECTIONAL    //全向轮的摩擦轮id和麦轮相反，左为1，右为2
        friction_config.can_init_config.tx_id = 1,
        friction_l = DJIMotorInit(&friction_config);

        friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
        friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
        friction_r = DJIMotorInit(&friction_config); 
#endif



 // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10,//10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 20000,
            },
            .speed_PID = {
                .Kp = 1.5,//0
                .Ki = 0.5,//0
                .Kd = 0.002,
                .Improve = PID_Integral_Limit |PID_ErrorHandle,
                .IntegralLimit = 200,
                .MaxOut = 10000,
            },
            .current_PID = {
                .Kp = 1.5,//0
                .Ki = 0.2,//0
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP|CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
	
}
//注意ShootTask任务的频率是200Hz
void ShootTask()
{

       // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

if (shoot_cmd_recv.Robot_Shoot_Status == ROBOT_STOP) // 关闭射击
{

        DJIMotorStop (friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);

}
else
{
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
}







    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        static uint8_t over_speed = 0;
        bullet_speed = shoot_cmd_recv.shoot_speed;
	    if(bullet_speed!=last_bullet_speed)
	    {
		    if(bullet_speed>23.5f)
		    {
			    over_speed++;
			    if(over_speed>=5)
			    {
				    over_speed=0;
				    fric_rate-=500;
			    }
		    }
	    }
       last_bullet_speed = bullet_speed;
	    if (change_speed < fric_rate)
        {
		change_speed+=160;
        // 前1秒内：参考值 = 30000 * 时间比例
        DJIMotorSetRef(friction_l, change_speed);
        DJIMotorSetRef(friction_r, change_speed);
        } 
		else
	   {
		change_speed=fric_rate;
        // 1秒后保持30000
        DJIMotorSetRef(friction_l, change_speed);
        DJIMotorSetRef(friction_r, change_speed);  
       }
    }
     else // 关闭摩擦轮
    {
        if (change_speed > 3000.0f)
        {
		change_speed-=160;
        // 前1秒内：参考值 = 30000 * 时间比例
        DJIMotorSetRef(friction_l, change_speed);
        DJIMotorSetRef(friction_r, change_speed);
        } 
		else
		{
	    change_speed=0;
	    DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
		}
    }

    switch (shoot_cmd_recv.load_mode)
    {
        
//      case LOAD_1_BULLET:
//       //  load_angle_set = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * 36;

//   //      DJIMotorOuterLoop(loader, SPEED_LOOP);    
//         if( loader->motor_controller.speed_PID.ERRORHandler.ERRORType==PID_MOTOR_BLOCKED_ERROR)
//         {   DJIMotorOuterLoop(loader, SPEED_LOOP);  
//             DJIMotorSetRef(loader,-shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
//         }   
//        else if (loader->motor_controller.speed_PID.ERRORHandler.ERRORType==PID_ERROR_NONE)
//        {           
//             DJIMotorOuterLoop(loader, SPEED_LOOP);  
//             DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8); // 控制量增加一发弹丸的角度
//        }
// 	   break;
	 case LOAD_STOP:
         DJIMotorOuterLoop(loader, SPEED_LOOP);    
		 DJIMotorSetRef(loader,0);
	 break;
	 
	 case LOAD_BURSTFIRE:
	   if( loader->motor_controller.speed_PID.ERRORHandler.ERRORType==PID_ERROR_NONE)
        {
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8); 
        }   
       else if (loader->motor_controller.speed_PID.ERRORHandler.ERRORType==PID_MOTOR_BLOCKED_ERROR)
       {
            DJIMotorSetRef(loader, -shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 10);
       }	 
    }


    shoot_feedback_data.friction_speed = friction_l->measure.speed_aps;

 PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
