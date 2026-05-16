#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "usbd_cdc_if.h"
#include "bsp_usb.h"
#include "master_process.h"
#include "bsp_dwt.h"
#include "buzzer.h"
#include "dm_imu.h"
#include "can.h"
#include "usart.h"
#include "vofa.h"

extern int16_t debug_idx1;

int16_t debug_yaw1[8];
int16_t debug_yaw2 = 0;
int16_t debug_yaw3 = 0;
int16_t debug_yaw6 = 0;
uint32_t debug_yaw4;
uint32_t debug_yaw5;
uint32_t control_timer;
float debug_gyro[10];

float debug_dt = 0, debug_t = 0;
static uint32_t Dt_Count = 0;
float debug_send_pitch_data = 180;
float debug_send_yaw_data = 0;

float debug_pitch_data[30];
float debug_yaw_data[30];

static uint32_t last_cnt = 0;

float debug_dm1;

static BuzzzerInstance *buzzer;

static imu_t *DM_IMU_DATA;   //达妙imu的数据，为双imu做准备
static attitude_t *gimba_IMU_data; // C板自带MU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Chassis_Upload_Data_s chassis_real_speed;
static Subscriber_t *chassis_speed_sub; // 底盘反馈信息订阅者

static float yaw_gyro_feedfofard = -0.472267985;
static float pitch_gyro_feedfofard = 0.5;

float pitch_Gravity_Compensation = 0;

float Pitch_Angle_Kp = 0.3, Pitch_Angle_Ki = 0, Pitch_Angle_Kd = 0;
float Pitch_speed_Kp = 4200, Pitch_Speed_Ki = 0, Pitch_Speed_Kd = 0;

float Yaw_Angle_Kp = 0.9, Yaw_Angle_Ki = 0.1, Yaw_Angle_Kd = 0.08;
float Yaw_speed_Kp = 1300, Yaw_Speed_Ki = 300, Yaw_Speed_Kd = 0;

/**
 * @brief 初始化云台
 */
void GimbalInit() {

    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    DM_IMU_DATA = imu_get_data();

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.9,
                .Ki = 0.1, 
                .Kd = 0.008, 
                .CoefA = 5, 
                .CoefB = 0.5, 
                .Output_LPF_RC = 0.001,
                .DeadBand = 0.2, 
                .IntegralSeparationThreshold = 40,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_OutputFilter | PID_ChangingIntegrationRate | PID_IntegralSeparation,
                .IntegralLimit = 0.5,
                .MaxOut = 1800,
            },
            .speed_PID = {
                .Kp = 1300, 
                .Ki = 300,
                .Kd = 0,
                .IntegralSeparationThreshold = 5,
                .Improve = PID_Integral_Limit | PID_IntegralSeparation,
                .IntegralLimit = 8000,
                .MaxOut = 16000,
            },
            .other_angle_feedback_ptr = &DM_IMU_DATA->yaw, // 注意这里是用的imu的yaw角度数据，单位是度
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, // 速度数据来源，注意使用的是编码器还是imu
            .speed_feedback_source = OTHER_FEED, // 位置数据来源，注意使用的是编码器还是imu
            .outer_loop_type = ANGLE_LOOP, // 外层闭环
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP, // 使用的是哪几个pid闭环
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 是否反转
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, // 反馈数据是否反向
        },
        .motor_type = GM6020,
    };

    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0, 
                .Ki = 0,
                .Kd = 0, 
                .DeadBand = 0.05, 
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                .IntegralLimit = 400,
                .MaxOut = 1000, 
            },
            .speed_PID = {
                .Kp = 0, 
                .Ki = 0,
                .Kd = 0, 
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 6000, 
                .MaxOut = 16000, 
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
            .Gravity_Compensation = &pitch_Gravity_Compensation,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_speed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

/**
 * @brief 动态改变pid的值，便于调参
 */
static void MotorSetAnglePIDParams(DJIMotorInstance *motor, float angle_kp, float angle_ki, float angle_kd, float speed_kp, float speed_ki, float speed_kd) {
    if (motor == NULL) return;
    // 直接修改电机实例中的PID参数
    motor->motor_controller.angle_PID.Kp = angle_kp;
    motor->motor_controller.angle_PID.Ki = angle_ki;
    motor->motor_controller.angle_PID.Kd = angle_kd;

    motor->motor_controller.speed_PID.Kp = speed_kp;
    motor->motor_controller.speed_PID.Ki = speed_ki;
    motor->motor_controller.speed_PID.Kd = speed_kd;
}

/**
 * @brief 云台任务处理函数
 *        注意：GimbalTask任务的频率是200Hz
 */
void GimbalTask() {
    debug_dt = DWT_GetDeltaT(&Dt_Count);
    debug_t += debug_dt;

    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_speed_sub, &chassis_real_speed);

    if (gimbal_cmd_recv.Robot_Gimbal_Status == ROBOT_STOP) {
        DJIMotorStop(pitch_motor);
        DJIMotorStop(yaw_motor);
    } else {
        DJIMotorStop(pitch_motor);
        DJIMotorStop(yaw_motor); // 启动yaw轴电机
    }

    MotorSetAnglePIDParams(pitch_motor, Pitch_Angle_Kp, Pitch_Angle_Ki, Pitch_Angle_Kd, Pitch_speed_Kp, Pitch_Speed_Ki, Pitch_Speed_Kd);
    MotorSetAnglePIDParams(yaw_motor, Yaw_Angle_Kp, Yaw_Angle_Ki, Yaw_Angle_Kd, Yaw_speed_Kp, Yaw_Speed_Ki, Yaw_Speed_Kd);

    debug_pitch_data[0] = pitch_motor->measure.speed_aps;
    debug_pitch_data[1] = pitch_motor->measure.real_current;
    debug_pitch_data[2] = pitch_motor->measure.angle_single_round;

    debug_pitch_data[3] = gimba_IMU_data->Gyro[0];
    debug_pitch_data[4] = gimba_IMU_data->Pitch;

    debug_pitch_data[5] = gimbal_cmd_recv.pitch;

    debug_pitch_data[6] = pitch_motor->motor_controller.angle_PID.Output; // 角度PID的输出
    debug_pitch_data[7] = pitch_motor->motor_controller.angle_PID.Pout; // 角度PID的比例项
    debug_pitch_data[8] = pitch_motor->motor_controller.angle_PID.Iout; // 角度PID的积分项
    debug_pitch_data[9] = pitch_motor->motor_controller.angle_PID.ITerm; // 角度PID的输出变化
    debug_pitch_data[10] = pitch_motor->motor_controller.angle_PID.Err; // 角度PID的误差值

    debug_pitch_data[11] = pitch_motor->motor_controller.speed_PID.Output; // 速度PID的输出
    debug_pitch_data[12] = pitch_motor->motor_controller.speed_PID.Pout; // 速度PID的比例项
    debug_pitch_data[13] = pitch_motor->motor_controller.speed_PID.Iout; // 速度PID的积分项
    debug_pitch_data[14] = pitch_motor->motor_controller.speed_PID.ITerm; // 速度PID的输出变化
    debug_pitch_data[15] = pitch_motor->motor_controller.speed_PID.Err; // 速度PID的误差值

    debug_pitch_data[16] = pitch_motor->measure.temperature; // 电机温度
    debug_pitch_data[17] = pitch_motor->measure.total_angle; // 电机总角度

    debug_yaw_data[0] = yaw_motor->measure.speed_aps;
    debug_yaw_data[1] = yaw_motor->measure.real_current;
    debug_yaw_data[2] = yaw_motor->measure.angle_single_round;

    debug_yaw_data[3] = gimba_IMU_data->Gyro[2]; // Yaw轴角速度
    debug_yaw_data[4] = gimba_IMU_data->Yaw; // Yaw角度

    debug_yaw_data[5] = gimbal_cmd_recv.yaw; // Yaw控制指令

    debug_yaw_data[6] = yaw_motor->motor_controller.angle_PID.Output; // 角度PID的输出
    debug_yaw_data[7] = yaw_motor->motor_controller.angle_PID.Pout; // 角度PID的比例项
    debug_yaw_data[8] = yaw_motor->motor_controller.angle_PID.Iout; // 角度PID的积分项
    debug_yaw_data[9] = yaw_motor->motor_controller.angle_PID.ITerm; // 角度PID的输出变化
    debug_yaw_data[10] = yaw_motor->motor_controller.angle_PID.Err; // 角度PID的误差值

    debug_yaw_data[11] = yaw_motor->motor_controller.speed_PID.Output; // 速度PID的输出
    debug_yaw_data[12] = yaw_motor->motor_controller.speed_PID.Pout; // 速度PID的比例项
    debug_yaw_data[13] = yaw_motor->motor_controller.speed_PID.Iout; // 速度PID的积分项
    debug_yaw_data[14] = yaw_motor->motor_controller.speed_PID.ITerm; // 速度PID的输出变化
    debug_yaw_data[15] = yaw_motor->motor_controller.speed_PID.Err; // 速度PID的误差值

    debug_yaw_data[16] = yaw_motor->measure.temperature; // 电机温度
    debug_yaw_data[17] = yaw_motor->measure.total_angle; // 电机总角度
    debug_yaw_data[18] = yaw_motor->motor_controller.angle_PID.Ref; // 速度PID的误差值
    debug_yaw_data[19] = yaw_motor->motor_controller.speed_PID.Ref; // 速度PID的误差值

    debug_yaw1[0] = yaw_motor->measure.speed_aps;
    debug_yaw2 = yaw_motor->measure.real_current;
    debug_yaw3 = yaw_motor->measure.ecd;

    debug_gyro[0] = gimba_IMU_data->Gyro[0];
    debug_gyro[1] = gimba_IMU_data->Gyro[1];
    debug_gyro[2] = gimba_IMU_data->Gyro[2];

    debug_gyro[3] = gimba_IMU_data->Roll;
//    debug_gyro[4] = gimba_IMU_data->Pitch;
    debug_gyro[5] = gimba_IMU_data->YawTotalAngle;

    debug_gyro[6] = gimbal_cmd_recv.pitch;
    debug_gyro[7] = gimbal_cmd_recv.yaw;

    // DJIMotorSetRef(pitch_motor, debug_send_pitch_data);
    DJIMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈，设置电机目标值，如果外层是角度环则设的是目标角度，速度同理，dji_motor.c里会处理好pid计算

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round; // 把yaw轴编码器的值发到处理中心，用于坐标系转化
    gimbal_feedback_data.yaw = gimba_IMU_data->Yaw;
    gimbal_feedback_data.pitch = gimba_IMU_data->Pitch;

    gimbal_feedback_data.Yaw_totalangle = gimba_IMU_data->YawTotalAngle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);


    VisionSetAltitude(DM_IMU_DATA->yaw,0, DM_IMU_DATA->pitch,0,21); // 发送当前姿态数据到视觉系统，数据来自达妙的imu
    VisionSetq(DM_IMU_DATA->q[0], DM_IMU_DATA->q[1], DM_IMU_DATA->q[2], DM_IMU_DATA->q[3]);

    float data[5] = {debug_pitch_data[4],DM_IMU_DATA->pitch,debug_gyro[4],gimbal_cmd_recv.yaw,debug_yaw_data[4]}; // 要发送的浮点数据
    vofa_justfloat_output(data, 5, &huart1); // 通过串口1发送

    float dt = DWT_GetDeltaT(&last_cnt);

    debug_gyro[4]=gimbal_cmd_recv.pitch*1.3f + 180.0f;
	
    if (gimbal_cmd_recv.yaw == 0)
    {
    debug_dm1 = DM_IMU_DATA->yaw;
    }
	debug_dm1 = gimbal_cmd_recv.yaw;
	
}
