#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stm32f4xx_hal.h"
#include "bsp_can.h"

/* ---------------- 数据范围宏 ---------------- */
#define ACCEL_CAN_MAX   (235.2f)
#define ACCEL_CAN_MIN   (-235.2f)
#define GYRO_CAN_MAX    (34.88f)
#define GYRO_CAN_MIN    (-34.88f)
#define PITCH_CAN_MAX   (90.0f)
#define PITCH_CAN_MIN   (-90.0f)
#define ROLL_CAN_MAX    (180.0f)
#define ROLL_CAN_MIN    (-180.0f)
#define YAW_CAN_MAX     (180.0f)
#define YAW_CAN_MIN     (-180.0f)
#define Quaternion_MIN  (-1.0f)
#define Quaternion_MAX  (1.0f)

/* ---------------- IMU 数据结构 ---------------- */
typedef struct {
    uint8_t  can_id;
    uint8_t  mst_id;
    CANInstance *can_instance;

    float pitch;
    float roll;
    float yaw;

    float gyro[3];
    float accel[3];

    float q[4];
    float cur_temp;
} imu_t;

/* ---------------- 对外公开接口（只有3个） ---------------- */
void IMU_RxCallback(CANInstance *instance);
void DM_IMU_Init(void);
imu_t* imu_get_data(void);

#endif /* __DM_IMU_H */