/**
 * @file  corner.h
 * @brief 直角转弯判断模块 — 全白检测 + 方向推断 + 转弯调度
 *
 * 直角判断逻辑:
 *   - 12路灰度全白 (PID_IsLineLost) → 判定到达直角
 *   - 上一帧线位置 (PID_GetFilteredPosition) 决定转弯方向:
 *       负值 = 线在左侧 → 左转 90°
 *       正值 = 线在右侧 → 右转 90°
 *   - 3帧去抖, 避免误触发
 *
 * 依赖:
 *   - grayscale.h : Grayscale_ReadAll()
 *   - pid.h       : PID_Update(), PID_IsLineLost(), PID_GetFilteredPosition()
 *   - turn.h      : Turn_Start(), Turn_Task(), Turn_IsBusy()
 *   - gyro.h      : GYRO_Poll() (需在主循环中 Corner_Task 之前调用)
 *
 * 主循环调用顺序 (关键 — 保证陀螺仪数据畅通):
 * @code
 *   while (1) {
 *       GYRO_Poll();     // 1. 高频轮询陀螺仪 UART, 绝不能阻塞
 *       Corner_Task();   // 2. 灰度循迹 + 直角检测 + 转弯
 *   }
 * @endcode
 */

#ifndef __CORNER_H__
#define __CORNER_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化直角转弯模块 (含 PID 和 Turn 子模块)
 * @param Kp, Ki, Kd  PID 控制参数
 * @param base_speed  基础电机速度 (0~100%)
 * @param trim        左右轮补偿 (正值=右转倾向, 补偿物理偏斜)
 */
void Corner_Init(float Kp, float Ki, float Kd, float base_speed, float trim);

/**
 * @brief 直角检测与循迹主任务 — 主循环中每轮调用
 * @note  调用前必须先调用 GYRO_Poll() 以更新陀螺仪数据
 *        本函数内部无阻塞, 不包含 delay, 快速返回
 */
void Corner_Task(void);

/**
 * @brief 查询当前是否正在执行直角转弯
 * @return true=转弯中, false=正在循迹
 */
bool Corner_IsTurning(void);

/**
 * @brief 获取已完成的直角转弯次数
 */
uint8_t Corner_GetCount(void);

#endif /* __CORNER_H__ */
