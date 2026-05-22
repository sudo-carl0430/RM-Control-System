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
#include <stdbool.h>

extern int16_t debug_idx1;

float Compensation = 0;
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
static attitude_t *gimbal_IMU_data; // C板自带MU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Chassis_Upload_Data_s chassis_real_speed;
static Subscriber_t *chassis_speed_sub; // 底盘反馈信息订阅者


static float yaw_last_cmd = 0.0f;         // 上一次的指令
static float yaw_filtered_cmd = 0.0f;     // 经过滤波的指令
static float yaw_cmd_rate = 0.0f;         // 指令变化率
static float yaw_static_error_accumulator = 0.0f;
static float yaw_filtered_cmd_prev = 0.0f;  // 用于二阶滤波
static float last_cmd_delta = 0.0f;         // 用于检测方向变化
static uint32_t last_direction_change_time = 0; // 方向变换时间戳
static bool direction_changed = false;      // 方向变化标志
static float estimated_frequency = 0.0f;    // 估计频率
static float peak_values[2] = {0.0f, 0.0f}; // 峰值记录


// 全局变量（放在函数外部）
static float last_target_deg = 0.0f;    // 上一时刻目标角度（度）
static float last_target_vel = 0.0f;    // 上一时刻目标速度（度/秒）
static float filtered_vel = 0.0f;       // 滤波后的目标速度
static float filtered_accel = 0.0f;     // 滤波后的目标加速度

static float yaw_gyro_feedfofard = 0;
static float pitch_gyro_feedfofard = 0.5;

float pitch_Gravity_Compensation = 0;

float Pitch_Angle_Kp = 0.3, Pitch_Angle_Ki = 0, Pitch_Angle_Kd = 0;
float Pitch_speed_Kp = 4200, Pitch_Speed_Ki = 0, Pitch_Speed_Kd = 0;

float Yaw_Angle_Kp = 0.5, Yaw_Angle_Ki = 0.1, Yaw_Angle_Kd = 0.08;
float Yaw_speed_Kp = 800, Yaw_Speed_Ki = 100, Yaw_Speed_Kd = 0;

/**
 * @brief 初始化云台
 */
void GimbalInit() {

    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
  // DM_IMU_DATA = imu_get_data();

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
                .Output_LPF_RC = 0.03,
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
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle, // 注意这里是用的imu的yaw角度数据，单位是度
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &yaw_gyro_feedfofard, // 注意这里是用的imu的陀螺仪数据，单位是度/s
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
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
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


// Yaw轴自适应跟随控制函数
void YawAdaptiveFollowControl() {
    float current_yaw = gimbal_IMU_data->YawTotalAngle;
    float yaw_cmd = gimbal_cmd_recv.yaw;

    // 计算指令变化量
    float cmd_delta = yaw_cmd - yaw_last_cmd;

    float yaw_error = yaw_cmd - current_yaw;
    float yaw_error_rate = -gimbal_IMU_data->Gyro[2];

    // ---------- 方向变化检测 & 频率估计 ----------
    if (cmd_delta * last_cmd_delta < 0.0f && fabsf(cmd_delta) > 0.1f) {
        direction_changed = true;
        float current_time = DWT_GetTimeline_ms();
        if (last_direction_change_time > 0.0f) {
            float period_ms = current_time - last_direction_change_time;
            if (period_ms > 100.0f) {
                estimated_frequency = 1000.0f / (period_ms * 2.0f);
                if (estimated_frequency > 2.0f)  estimated_frequency = 2.0f;
                if (estimated_frequency < 0.05f) estimated_frequency = 0.05f;
            }
        }
        last_direction_change_time = current_time;
    } else {
        float current_time = DWT_GetTimeline_ms();
        if (current_time - last_direction_change_time > 200.0f) {
            direction_changed = false;
        }
    }

    // ---------- 动态滤波系数 ----------
    float filter_factor = 0.85f;  // 默认

    if (fabsf(cmd_delta) > 20.0f) {
        filter_factor = 0.6f;
    } else if (direction_changed) {
        filter_factor = 0.55f;
    } else if (fabsf(cmd_delta) > 0.4f) {
        filter_factor = 0.8f;
    } else {
        filter_factor = 0.95f;
    }

    // ---------- 静态误差积分 ----------
    if (fabsf(cmd_delta) < 0.2f) {
        if (fabsf(yaw_error) > 0.3f && fabsf(yaw_error_rate) < 0.05f) {
            yaw_static_error_accumulator += yaw_error * 0.003f;
            yaw_static_error_accumulator = fmaxf(-2.0f, fminf(yaw_static_error_accumulator, 2.0f));
        }
    }

    // ---------- 指令滤波（积分不参与滤波） ----------
    yaw_filtered_cmd = filter_factor * yaw_cmd +
                       (1.0f - filter_factor) * yaw_filtered_cmd;

    float target = yaw_filtered_cmd + yaw_static_error_accumulator;

    // 设置电机位置目标
    DJIMotorSetRef(yaw_motor, target);

    // ---------- 速度前馈计算 ----------
    float feed_forward = 0.0f;

    if (fabsf(cmd_delta) > 0.3f && fabsf(cmd_delta) < 5.0f) {
        feed_forward = cmd_delta * 150.0f;
        if (estimated_frequency > 0.3f) {
            feed_forward *= (1.0f + 1.0f * estimated_frequency);
        }
    } else if (fabsf(cmd_delta) >= 5.0f) {
        feed_forward = yaw_error * 1.5f;
    } else {
        if (fabsf(yaw_error) > 0.2f) {
            feed_forward = yaw_error * 1.0f;
        }
    }

    if (direction_changed) {
        feed_forward *= 0.5f;
    }

    // 你的新前馈出口：赋值给全局变量，而不是直接调DMMotorSetSpeedFeedForward
    yaw_gyro_feedfofard = feed_forward;

    // 更新跟踪变量
    yaw_last_cmd = yaw_cmd;
    last_cmd_delta = cmd_delta;
}

float ImprovedGravityCompensation(float dt)
{
    // ==================== 0. 获取目标角度（度） ====================
    float target_deg = gimbal_cmd_recv.pitch;  // 设定值，单位：度
    
    // ==================== 1. 重力前馈（基于目标角度） ====================
    float target_rad = target_deg * DEGREE_2_RAD;   // 度转弧度
    
    // 你的拟合参数（请根据实测调整）
    const float A = -651.13f;
    const float k = 9411.53f;
    const float y = -2.1065f;          // 弧度
    const float lambda = 0.9f;         // 前馈强度（0~1，给 PID 留余量）
    
    // 计算重力补偿值（此时单位可能是拟合直接对应的 DAC 值或电流值）
    // 假设你的原始公式得到的就是 DAC 值，如果是电流则需另外转换
    float gravity_dac = (A + k * arm_cos_f32(target_rad + y)) * lambda;
    
    // ==================== 2. 惯性力前馈（基于目标加速度） ====================
    // ---- 2.1 计算目标速度（度/秒）----
    float raw_vel = (target_deg - last_target_deg) / dt;
    
    // ---- 2.2 一阶低通滤波（抑制差分噪声）----
    const float alpha_vel = 0.7f;   // 滤波系数，越大越灵敏，越小越平滑
    filtered_vel = alpha_vel * filtered_vel + (1.0f - alpha_vel) * raw_vel;
    
    // ---- 2.3 计算目标加速度（度/秒²）----
    float raw_accel = (filtered_vel - last_target_vel) / dt;
    
    // ---- 2.4 加速度也滤波 ----
    const float alpha_acc = 0.5f;
    filtered_accel = alpha_acc * filtered_accel + (1.0f - alpha_acc) * raw_accel;
    
    // 更新历史值
    last_target_deg = target_deg;
    last_target_vel = filtered_vel;
    
    // ---- 2.5 转换为弧度加速度，并乘以转动惯量系数 ----
    float accel_rad = filtered_accel * DEGREE_2_RAD;   // rad/s²
    
    // ★★★ 关键参数：需要你标定！ ★★★
    // J_to_dac 的作用：把 (rad/s²) 直接映射成 DAC 指令值
    // 推导：J(kg·m²) * accel(rad/s²) = 力矩(Nm)
    //      电流(A) = 力矩 / 电机力矩常数 Kt (Nm/A)
    //      DAC值 = 电流 * (16384 / 3)
    // 所以 J_to_dac = J * (16384.0f / 3.0f) / Kt
    // 暂时设成 0 可以禁用惯性前馈，等你标定好再填。
    float J_to_dac = 3.252f;   // TODO: 标定后填入，例如 120.5f
    
    float inertia_dac = J_to_dac * accel_rad;
    
    // ==================== 3. 总前馈 ====================
    float total_ff_dac = gravity_dac + inertia_dac;
    
    // ---- 安全限幅（防止异常值超出 DAC 范围）----
    if (total_ff_dac > 16384.0f)  total_ff_dac = 16384.0f;
    if (total_ff_dac < -16384.0f) total_ff_dac = -16384.0f;
    
    return total_ff_dac;
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
        DJIMotorEnable(pitch_motor);
        DJIMotorEnable(yaw_motor); // 启动yaw轴电机
    }

    MotorSetAnglePIDParams(pitch_motor, Pitch_Angle_Kp, Pitch_Angle_Ki, Pitch_Angle_Kd, Pitch_speed_Kp, Pitch_Speed_Ki, Pitch_Speed_Kd);
    MotorSetAnglePIDParams(yaw_motor, Yaw_Angle_Kp, Yaw_Angle_Ki, Yaw_Angle_Kd, Yaw_speed_Kp, Yaw_Speed_Ki, Yaw_Speed_Kd);

    debug_pitch_data[0] = pitch_motor->measure.speed_aps;
    debug_pitch_data[1] = pitch_motor->measure.real_current;
    debug_pitch_data[2] = pitch_motor->measure.angle_single_round;

    debug_pitch_data[3] = gimbal_IMU_data->Gyro[0];
    debug_pitch_data[4] = gimbal_IMU_data->Pitch;

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

    debug_yaw_data[3] = gimbal_IMU_data->Gyro[2]; // Yaw轴角速度
    debug_yaw_data[4] = gimbal_IMU_data->Yaw; // Yaw角度

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

    debug_gyro[0] = gimbal_IMU_data->Gyro[0];
    debug_gyro[1] = gimbal_IMU_data->Gyro[1];
    debug_gyro[2] = gimbal_IMU_data->Gyro[2];

    debug_gyro[3] = gimbal_IMU_data->Roll;
//    debug_gyro[4] = gimba_IMU_data->Pitch;
    debug_gyro[5] = gimbal_IMU_data->YawTotalAngle;

    debug_gyro[6] = gimbal_cmd_recv.pitch;
    debug_gyro[7] = gimbal_cmd_recv.yaw;

    // DJIMotorSetRef(pitch_motor, debug_send_pitch_data);
    DJIMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈，设置电机目标值，如果外层是角度环则设的是目标角度，速度同理，dji_motor.c里会处理好pid计算

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round; // 把yaw轴编码器的值发到处理中心，用于坐标系转化
    gimbal_feedback_data.yaw = gimbal_IMU_data->Yaw;
    gimbal_feedback_data.pitch = gimbal_IMU_data->Pitch;

    gimbal_feedback_data.Yaw_totalangle = gimbal_IMU_data->YawTotalAngle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);


    // VisionSetAltitude(DM_IMU_DATA->yaw,0, DM_IMU_DATA->pitch,0,21); // 发送当前姿态数据到视觉系统，数据来自达妙的imu
    // VisionSetq(DM_IMU_DATA->q[0], DM_IMU_DATA->q[1], DM_IMU_DATA->q[2], DM_IMU_DATA->q[3]);

//    float data[5] = {debug_pitch_data[4],DM_IMU_DATA->pitch,debug_gyro[4],gimbal_cmd_recv.yaw,debug_yaw_data[4]}; // 要发送的浮点数据
//    vofa_justfloat_output(data, 5, &huart1); // 通过串口1发送



    Compensation = ImprovedGravityCompensation(0.001f);
    pitch_Gravity_Compensation = ImprovedGravityCompensation(0.001f);

//    YawAdaptiveFollowControl();
    float data[9] = {1.23f,gimbal_IMU_data->Pitch,gimbal_cmd_recv.pitch,pitch_motor->measure.real_current,pitch_motor->motor_controller.angle_PID.Output,pitch_motor->motor_controller.speed_PID.Output}; // 要发送的浮点数据
    vofa_justfloat_output(data,9, &huart1); // 通过串口1发送

// float data[9] = {1.23f,gimbal_IMU_data->YawTotalAngle,gimbal_cmd_recv.yaw,yaw_motor->measure.real_current,yaw_motor->motor_controller.angle_PID.Output,yaw_motor->motor_controller.speed_PID.Output};
// vofa_justfloat_output(data,9, &huart1); // 通过串口1发送


    float dt = DWT_GetDeltaT(&last_cnt);
	
}
