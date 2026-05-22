/**
 * @file alg_slope.h
 * @brief 斜坡函数（C 语言版本），用于速度规划等
 */

#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 枚举类型：规划优先类型 */
typedef enum {
    SLOPE_FIRST_REAL = 0,    // 真实值优先
    SLOPE_FIRST_TARGET       // 目标值优先
} Slope_First_t;

/* 斜坡函数结构体 */
typedef struct {
    // 配置参数
    float increase_value;      // 一次计算周期允许增加的最大绝对值
    float decrease_value;      // 一次计算周期允许减少的最大绝对值
    Slope_First_t slope_first; // 优先模式

    // 状态变量
    float now_planning;        // 当前规划值
    float now_real;            // 当前真实值
    float target;              // 目标值

    // 输出值
    float out;
} Class_Slope_t;

/**
 * @brief 初始化斜坡函数对象
 * @param slope   斜坡对象指针
 * @param increase 增加步长（绝对值）
 * @param decrease 减少步长（绝对值）
 * @param first   优先模式
 */
void Class_Slope_Init(Class_Slope_t *slope, float increase, float decrease, Slope_First_t first);

/**
 * @brief 获取当前输出值
 * @param slope 斜坡对象指针
 * @return 输出值
 */
static inline float Class_Slope_Get_Out(Class_Slope_t *slope) {
    return slope->out;
}

/**
 * @brief 设置当前真实值
 * @param slope 斜坡对象指针
 * @param real  真实值
 */
static inline void Class_Slope_Set_Now_Real(Class_Slope_t *slope, float real) {
    slope->now_real = real;
}

/**
 * @brief 设置增加步长
 * @param slope  斜坡对象指针
 * @param inc    增加步长
 */
static inline void Class_Slope_Set_Increase_Value(Class_Slope_t *slope, float inc) {
    slope->increase_value = inc;
}

/**
 * @brief 设置减少步长
 * @param slope  斜坡对象指针
 * @param dec    减少步长
 */
static inline void Class_Slope_Set_Decrease_Value(Class_Slope_t *slope, float dec) {
    slope->decrease_value = dec;
}

/**
 * @brief 设置目标值
 * @param slope  斜坡对象指针
 * @param target 目标值
 */
static inline void Class_Slope_Set_Target(Class_Slope_t *slope, float target) {
    slope->target = target;
}

/**
 * @brief 执行一次斜坡计算（应在周期循环中调用）
 * @param slope 斜坡对象指针
 */
void Class_Slope_TIM_Calculate_PeriodElapsedCallback(Class_Slope_t *slope);

#ifdef __cplusplus
}
#endif

#endif /* ALG_SLOPE_H */
