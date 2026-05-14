/**
 * @file alg_slope.c
 * @brief 斜坡函数实现（C 语言版本）
 */

#include "alg_slope.h"
#include <math.h>   // 用于 fabsf

/* 内部辅助函数：绝对值（float）*/
static inline float Math_Abs(float x) {
    return fabsf(x);
}

/**
 * @brief 初始化斜坡函数对象
 */
void Class_Slope_Init(Class_Slope_t *slope, float increase, float decrease, Slope_First_t first) {
    slope->increase_value = increase;
    slope->decrease_value = decrease;
    slope->slope_first = first;
    slope->now_planning = 0.0f;
    slope->now_real = 0.0f;
    slope->target = 0.0f;
    slope->out = 0.0f;
}

/**
 * @brief 执行一次斜坡计算（应在周期循环中调用）
 */
void Class_Slope_TIM_Calculate_PeriodElapsedCallback(Class_Slope_t *slope) {
    // 1. 真实值优先模式特殊处理
    if (slope->slope_first == SLOPE_FIRST_REAL) {
        // 如果真实值位于规划值与目标值之间（包括等于），则输出直接等于真实值
        if ((slope->target >= slope->now_real && slope->now_real >= slope->now_planning) ||
            (slope->target <= slope->now_real && slope->now_real <= slope->now_planning)) {
            slope->out = slope->now_real;
        }
    }

    // 2. 根据当前规划值和目标值的关系，计算新的输出值
    if (slope->now_planning > 0.0f) {
        if (slope->target > slope->now_planning) {
            // 正值加速
            if (Math_Abs(slope->now_planning - slope->target) > slope->increase_value) {
                slope->out += slope->increase_value;
            } else {
                slope->out = slope->target;
            }
        } else if (slope->target < slope->now_planning) {
            // 正值减速
            if (Math_Abs(slope->now_planning - slope->target) > slope->decrease_value) {
                slope->out -= slope->decrease_value;
            } else {
                slope->out = slope->target;
            }
        }
    } else if (slope->now_planning < 0.0f) {
        if (slope->target < slope->now_planning) {
            // 负值加速（向更负的方向）
            if (Math_Abs(slope->now_planning - slope->target) > slope->increase_value) {
                slope->out -= slope->increase_value;
            } else {
                slope->out = slope->target;
            }
        } else if (slope->target > slope->now_planning) {
            // 负值减速（向零靠近）
            if (Math_Abs(slope->now_planning - slope->target) > slope->decrease_value) {
                slope->out += slope->decrease_value;
            } else {
                slope->out = slope->target;
            }
        }
    } else { // now_planning == 0.0f
        if (slope->target > slope->now_planning) {
            // 从 0 正向加速
            if (Math_Abs(slope->now_planning - slope->target) > slope->increase_value) {
                slope->out += slope->increase_value;
            } else {
                slope->out = slope->target;
            }
        } else if (slope->target < slope->now_planning) {
            // 从 0 负向加速
            if (Math_Abs(slope->now_planning - slope->target) > slope->increase_value) {
                slope->out -= slope->increase_value;
            } else {
                slope->out = slope->target;
            }
        }
    }

    // 3. 更新规划值
    slope->now_planning = slope->out;
}