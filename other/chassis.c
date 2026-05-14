#include "chassis.h"
#include "robot_cmd.h"
#include "dji_motor.h"
#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "motor_def.h"
#include "bsp_can.h"
#include "math.h"
#include "power_control.h"
#include "usart.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "master_process.h"
#include "alg_slope.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

float debug[10];

static uint32_t Dt_Count = 0;
float change_dt = 0, change_t = 0;

float debug_speed;
float Chassis_follow_lapse;

#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif

static Class_Slope_t slope_vx;
static Class_Slope_t slope_vy;
static Class_Slope_t slope_wz;

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static referee_info_t* referee_data; // 用于获取裁判系统的数据

static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static float chassis_SPEED_PID[3] = {4.5f, 0.0f, 0.0f};
static float chassis_CURRENT_PID[3] = {0.5f, 0.0f, 0.0f};

float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static float rotate_speed_buff = 0;

float chassis_motor_speed[4];
int16_t motor_speed_pid_out[4];

static float power_buff; // 缓冲能量
static float limit_power; // 限制的功率
static float limit_error;
static float referee_real_power; // 缓冲的功率
static int32_t limit_cnt;
static uint16_t referee_limit_power;
uint8_t Is_power_all_out;
uint8_t power_cmd_send[8] = {0};

/**
 * @brief 根据robot_def.h中的macro自动计算的参数
 *        半轴距
 */
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/**
 * @brief 初始化底盘
 */
void ChassisInit() {
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = chassis_SPEED_PID[0], // 4.5
                .Ki = chassis_SPEED_PID[1], // 0
                .Kd = chassis_SPEED_PID[2], // 0
                .IntegralLimit = 3000,
                .Output_LPF_RC = 0.02,
                .DeadBand = 20,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = chassis_CURRENT_PID[0], // 0.4
                .Ki = chassis_CURRENT_PID[1], // 0
                .Kd = chassis_CURRENT_PID[2],
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

#ifdef ROBOT_MECANUM
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = PowerControlInit(&chassis_motor_config);
#endif

#ifdef ROBOT_OMNIDIRECTIONAL
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_motor_config);
#endif

    referee_data = Referee_Interactive_init(&huart6, &ui_data); // 裁判系统初始化

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void Kinematics_calculation() {
#ifdef ROBOT_MECANUM
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
#endif

#ifdef ROBOT_OMNIDIRECTIONAL
    const float SQRT2_2 = 0.70710678118f;  // √2/2 = 0.70710678118
    vt_lf = chassis_vx * SQRT2_2 + chassis_vy * SQRT2_2 + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx * SQRT2_2 + chassis_vy * SQRT2_2 - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx * SQRT2_2 - chassis_vy * SQRT2_2 + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx * SQRT2_2 - chassis_vy * SQRT2_2 - chassis_cmd_recv.wz * RB_CENTER;
#endif
}

/**
 * @brief 电机功率限制计算并设置电机参考值
 */
void chassis_power_control() {
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 通过读取缓冲能量，实时修改功率限制的误差值，尽可能利用到缓冲能量
 */
static void LimitPowerCalculate() {
    power_buff = chassis_feedback_data.chassis_power_buffer;

    if (power_buff >= 60) { // 缓冲能量未被消耗，缓慢增加限制
        limit_cnt++;
        if (limit_cnt >= 4000) {
            limit_cnt = 4000;
            limit_error += 0; // 此处原意为限制不再增加，可改为 limit_error = limit_error; 或保持不变
        } else {
            limit_error = limit_cnt * 0.01;
        }
    } else if (40 < power_buff && power_buff < 60) { // 理想区间，误差清零
        limit_error = 0;
        limit_cnt = 0;
    } else if (power_buff <= 40) { // 缓冲能量不足，快速降低限制
        limit_error -= 1.5f * (40 - power_buff);
        limit_cnt = 0;
    }

    referee_limit_power = referee_data->GameRobotState.chassis_power_limit;
    power_cmd_send[0] = referee_limit_power - 10;

    // 直接使用裁判系统限制减去10再加上误差作为最终功率限制
    float final_limit = 120 - 10 + limit_error;
    if (final_limit < 0) {
        SetPowerLimit(50); // 安全值
    } else {
        SetPowerLimit(final_limit);
    }
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 */
static void EstimateSpeed() {
    chassis_feedback_data.real_wz = (motor_lf->measure.speed_aps + motor_lb->measure.speed_aps +
                                    motor_rf->measure.speed_aps + motor_rb->measure.speed_aps) 
                                    / 4 / 19 / 170 * 60 / 2;
}

/**
 * @brief 更新跟随角度
 */
static void update_chassis_follow() {
    float dt = change_dt;   // 注意：change_dt 已在外部通过 DWT_GetDeltaT 获得
    float target = chassis_cmd_recv.offset_angle;
    float diff = target - Chassis_follow_lapse;

    if (fabsf(diff) < 1e-6f) {
        Chassis_follow_lapse = target;
        return;
    }

    float max_step = 50000 * dt;

    if (fabsf(diff) <= max_step) {
        Chassis_follow_lapse = target;
    } else {
        Chassis_follow_lapse += (diff > 0 ? max_step : -max_step);
    }
}

/**
 * @brief 底盘任务处理函数
 *        注意：ChassisTask任务的频率是200Hz
 */
void ChassisTask() {
    change_dt = DWT_GetDeltaT(&Dt_Count);
    change_t += change_dt;

#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

    if (chassis_cmd_recv.Robot_Chassis_Status == ROBOT_STOP) {
        // 正常工作
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else {
        // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        // DJIMotorEnable(motor_rb);
        DJIMotorEnable(motor_rb);
    }

    update_chassis_follow();

#ifdef ROBOT_MECANUM
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_ROTATE: {
            float linear_speed = sqrtf(chassis_cmd_recv.vx * chassis_cmd_recv.vx +
                                       chassis_cmd_recv.vy * chassis_cmd_recv.vy);
            chassis_cmd_recv.wz = (linear_speed >= 100.0f) ? 2700.0f : 3500.0f;
        }
        break;

        case CHASSIS_NO_FOLLOW:      // 不跟随，允许全向平移
            chassis_cmd_recv.wz = 0;
            break;

        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
            chassis_cmd_recv.wz = -chassis_cmd_recv.offset_angle * 35;
            break;

        default:
            break;
    }
#endif

#ifdef ROBOT_OMNIDIRECTIONAL
    switch (chassis_cmd_recv.chassis_mode) 
    {
        case CHASSIS_ROTATE: 
        {
            float linear_speed = sqrtf(chassis_cmd_recv.vx * chassis_cmd_recv.vx +
                                       chassis_cmd_recv.vy * chassis_cmd_recv.vy);
            chassis_cmd_recv.wz = (linear_speed >= 100.0f) ? 2700.0f : 3700.0f;
        }
        break;


        case CHASSIS_ROTATE_REVERSE: 
        {
            float linear_speed = sqrtf(chassis_cmd_recv.vx * chassis_cmd_recv.vx +
                                       chassis_cmd_recv.vy * chassis_cmd_recv.vy);
            chassis_cmd_recv.wz = (linear_speed >= 100.0f) ? -2700.0f : -3700.0f;
        }
        break;


        case CHASSIS_NO_FOLLOW:      // 不跟随，允许全向平移
            chassis_cmd_recv.wz = 0;
            break;

        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
            chassis_cmd_recv.wz = (chassis_cmd_recv.offset_angle > 30) ? 
                                  chassis_cmd_recv.offset_angle * 30.0f : 
                                  chassis_cmd_recv.offset_angle * 50.0f;
            break;

        default:
            break;
    }
#endif

    static float sin_theta, cos_theta;
#ifdef ROBOT_MECANUM
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
#endif

#ifdef ROBOT_OMNIDIRECTIONAL
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD - chassis_cmd_recv.wz * DEGREE_2_RAD * 0.0046f);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD - chassis_cmd_recv.wz * DEGREE_2_RAD * 0.004f);
    // 补偿全向轮小陀螺时走直线会偏移
#endif

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出,包涵麦轮运动学解算和全向轮运动学解算
    Kinematics_calculation();

    // 根据电机的反馈值计算当前功率并进行限制
    LimitPowerCalculate();

    // 设定闭环参考值
    chassis_power_control();

    // 根据电机的反馈速度和IMU(如果有)计算真实旋转速度
    EstimateSpeed();

    // 设置裁判系统数据
    chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;
    chassis_feedback_data.chassis_power_buffer = referee_data->PowerHeatData.chassis_power_buffer;
    chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;

    // 设置UI界面
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.robot_status = chassis_cmd_recv.Robot_Chassis_Status;

    ui_data.Pitch_angle = 185.0f - chassis_cmd_recv.pitch_angle;

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
}
