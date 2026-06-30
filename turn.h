/**
 * @file  turn.h
 * @brief 两轮差速小车原地直角转弯模块 — 状态机 + P控制闭环
 *
 * 依赖:
 * - motor.h : Motor_SetSpeed(), Motor_Stop()
 * - gyro.h  : GYRO_GetYaw(), GYRO_GetWz()
 *
 * 使用示例:
 * @code
 *   int main(void) {
 *     SYSCFG_DL_init();
 *     Motor_Init();
 *     GYRO_Init();
 *     Turn_Init();
 *
 *     Turn_Start(90);   // 右转 90°
 *
 *     while (1) {
 *       GYRO_Poll();     // 高频轮询陀螺仪数据
 *       Turn_Task();     // 状态机驱动转弯
 *     }
 *   }
 * @endcode
 */

#ifndef TURN_H_
#define TURN_H_

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化转弯模块状态。
 * @note  调用前需确保 Motor_Init() 和 GYRO_Init() 已完成。
 */
void Turn_Init(void);

/**
 * @brief 启动一次原地转弯。
 * @param angle 相对角度增量 (°)
 *              - 正值: 顺时针 (右转)
 *              - 负值: 逆时针 (左转)
 *              - 典型值: ±90, ±180
 * @note  若上一轮转弯尚未完成 (Turn_IsBusy() == true), 本次调用将被忽略。
 */
void Turn_Start(int angle);

/**
 * @brief 转弯状态机任务, 需在主循环中高频调用。
 * @note  调用前必须先调用 GYRO_Poll() 以更新陀螺仪数据。
 *        本函数内部不使用 delay, 无阻塞。
 */
void Turn_Task(void);

/**
 * @brief 查询转弯是否正在进行中。
 * @return true  正在转弯 (不可接受新指令)
 * @return false 空闲, 可接受 Turn_Start()
 */
bool Turn_IsBusy(void);

#endif /* TURN_H_ */
