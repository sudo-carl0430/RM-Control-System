#include "dm_imu.h"
#include <string.h>
#include "bsp_dwt.h"

/* ========== 内部函数前置声明（全部为 static） ========== */
static void IMU_UpdateAccel(uint8_t* pData);
static void IMU_UpdateGyro(uint8_t* pData);
static void IMU_UpdateEuler(uint8_t* pData);
static void IMU_UpdateQuaternion(uint8_t* pData);
static void IMU_UpdateData(uint8_t* pData);

static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data);
static CANInstance* imu_register(CAN_HandleTypeDef *can_handle, uint32_t tx_id, uint32_t rx_id);
static void imu_write_reg(uint8_t reg_id, uint32_t data);
static void imu_read_reg(uint8_t reg_id);

static void imu_change_to_active(void);
static void imu_set_active_mode_delay(uint32_t delay_ms);
// 其他未用到的配置函数也可以保留为 static，这里只保留 DM_IMU_Init 中用到的

/* ================================================ */

static imu_t imu;   // 模块内部静态变量，外部不可见

/* ---------- 转换工具（内部使用）---------- */
static int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ---------- 发送命令（内部）---------- */
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data) {
    if (imu.can_instance == NULL) return;
    imu.can_instance->tx_buff[0] = 0xCC;
    imu.can_instance->tx_buff[1] = reg_id;
    imu.can_instance->tx_buff[2] = ac;
    imu.can_instance->tx_buff[3] = 0xDD;
    memcpy(&imu.can_instance->tx_buff[4], &data, 4);
    CANSetDLC(imu.can_instance, 8);
    CANTransmit(imu.can_instance, 5);
}

/* ---------- 注册（内部）---------- */
static CANInstance* imu_register(CAN_HandleTypeDef *can_handle, uint32_t tx_id, uint32_t rx_id) {
    CAN_Init_Config_s config;
    config.can_handle          = can_handle;
    config.tx_id               = tx_id;
    config.rx_id               = rx_id;
    config.can_module_callback = IMU_RxCallback;
    config.id                  = &imu;
    CANInstance *instance = CANRegister(&config);
    if (instance != NULL) {
        imu.can_id       = tx_id;
        imu.mst_id       = rx_id;
        imu.can_instance = instance;
    }
    return instance;
}

/* ---------- 基本读写（内部）---------- */
static void imu_write_reg(uint8_t reg_id, uint32_t data) {
    imu_send_cmd(reg_id, 0x01, data);   // 0x01 = CMD_WRITE
}
static void imu_read_reg(uint8_t reg_id) {
    imu_send_cmd(reg_id, 0x00, 0);      // 0x00 = CMD_READ
}

/* ---------- 模式切换（内部）---------- */
static void imu_change_to_active(void) {
    imu_write_reg(0x0C, 1);             // 0x0C = CHANGE_ACTIVE（根据原枚举）
}
static void imu_set_active_mode_delay(uint32_t delay_ms) {
    imu_write_reg(0x0B, delay_ms);      // 0x0B = SET_DELAY
}

/* ========== 一键初始化，在这设置哪条can，以及对应的标识符 ========== */
void DM_IMU_Init(void) {
    imu_register(&hcan1, 0x11, 0x12);
    imu_change_to_active();
    imu_set_active_mode_delay(1);   // 1ms → 1000Hz
}

/* ========== 对外公开：接收回调 ========== */
void IMU_RxCallback(CANInstance *instance) {
    IMU_UpdateData(instance->rx_buff);
}

/* ========== 数据解析（内部） ========== */
static void IMU_UpdateAccel(uint8_t* pData) {
    uint16_t accel[3];
    imu.cur_temp = pData[1];
    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];
    imu.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

static void IMU_UpdateGyro(uint8_t* pData) {
    uint16_t gyro[3];
    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];
    imu.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

static void IMU_UpdateEuler(uint8_t* pData) {
    int euler[3];
    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];
    imu.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    imu.yaw   = uint_to_float(euler[1], YAW_CAN_MIN,   YAW_CAN_MAX,   16);
    imu.roll  = uint_to_float(euler[2], ROLL_CAN_MIN,  ROLL_CAN_MAX,  16);
}

static void IMU_UpdateQuaternion(uint8_t* pData) {
    int w =  pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
    int z = (pData[6] & 0x3F) << 8  |  pData[7];
    imu.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

static void IMU_UpdateData(uint8_t* pData) {
    switch (pData[0]) {
        case 1: IMU_UpdateAccel(pData);      break;
        case 2: IMU_UpdateGyro(pData);       break;
        case 3: IMU_UpdateEuler(pData);      break;
        case 4: IMU_UpdateQuaternion(pData); break;
        default: break;
    }
}

/* ========== 对外公开：获取数据指针 ========== */
 imu_t* imu_get_data(void) {
    return &imu;
}