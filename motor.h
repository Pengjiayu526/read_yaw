/**
 * @file  motor.h
 * @brief 双轮电机驱动模块 — 通过 PWM 占空比控制转速
 *
 * 硬件连接:
 * - 通道0 (CCP0 / PA12) → 右轮
 * - 通道1 (CCP1 / PA13) → 左轮
 *
 * 使用示例:
 * @code
 *   SYSCFG_DL_init();
 *   Motor_Init();
 *   Motor_SetSpeed(50, 50);   // 两轮均为 50% 占空比
 *   Motor_SetSpeed(0, 60);    // 右轮停止, 左轮 60% (原地右转)
 *   Motor_Stop();             // 全部停止
 * @endcode
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

/*===========================================================================
 * 常量定义
 *===========================================================================*/

/** @brief PWM 周期 (定时器重装载值) — 占空比分辨率 0 ~ 128 */
#define MOTOR_PWM_PERIOD    128U

/** @brief 速度百分比上限 */
#define MOTOR_SPEED_MAX     100

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化电机 PWM 并启动输出。
 * @note  必须在 SYSCFG_DL_init() 之后调用。
 */
void Motor_Init(void);

/**
 * @brief 同时设置左右两轮的速度 (占空比百分比)。
 * @param leftSpeed  左轮速度 (0 = 停止, 100 = 全速)
 * @param rightSpeed 右轮速度 (0 = 停止, 100 = 全速)
 *
 * 通过改变 PWM 波占空比来控制电机转速:
 * - 占空比越高, 电机两端等效平均电压越高, 转速越快
 * - 传入 0 则对应通道输出 0% 占空比, 电机停转
 */
void Motor_SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed);

/**
 * @brief 停止两个电机 (等价于 Motor_SetSpeed(0, 0))。
 */
void Motor_Stop(void);

#endif /* MOTOR_H_ */
