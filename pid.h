/**
 * @file  pid.h
 * @brief PID 循迹控制器 — 加权位置计算 + EMA滤波 + PID核心 + 电机控制
 *        PID Line-Following Controller with weighted position,
 *        EMA filtering, full PID computation, and motor output.
 *
 * 硬件依赖:
 * - 12路灰度传感器 (Grayscale 模块)
 * - 双轮 PWM 电机驱动 (Motor 模块)
 *
 * 算法说明:
 * - 加权位置: 每个传感器赋予一个物理位置权重, 加权平均得到连续线位置
 * - EMA 低通滤波: 对位置和微分项分别做指数移动平均, 抑制传感器噪声
 * - PID 控制: P(比例) + I(积分/抗饱和) + D(微分/滤波)
 * - 差速转向: 输出修正量以差速方式分配至左右电机
 *
 * 使用示例:
 * @code
 *   SYSCFG_DL_init();
 *   USART_Init();
 *   Motor_Init();
 *   PID_Init(1.0f, 0.0f, 0.5f, 50.0f);   // Kp=1.0, Ki=0, Kd=0.5, 基础速度50%
 *
 *   while (1) {
 *       uint16_t sensors = Grayscale_ReadAll();
 *       PID_Update(sensors);               // 读取传感器 -> PID -> 控制电机
 *       delay_ms(5);                       // 控制周期 ~5ms (200Hz)
 *   }
 * @endcode
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/*===========================================================================
 * 常量定义
 *===========================================================================*/

/** @brief 灰度传感器通道数 */
#define PID_SENSOR_COUNT    12U

/** @brief 线位置有效范围 (权重最大偏移量) */
#define PID_POSITION_MAX    5.5f

/*===========================================================================
 * 函数声明 — 核心 API
 *===========================================================================*/

/**
 * @brief 初始化 PID 循迹控制器。
 *        Initialize PID line-following controller.
 * @param Kp         比例增益  Proportional gain
 * @param Ki         积分增益  Integral gain
 * @param Kd         微分增益  Derivative gain
 * @param base_speed 基础电机速度 (0~100%)  Base motor speed
 * @param trim       左右轮补偿 (0~100%)  正值=左轮加速/右轮减速 → 右转倾向
 *                   补偿物理偏斜: 车老偏左设正值, 老偏右设负值, 典型 2~8
 * @note  调用前必须先初始化 Motor 模块 (Motor_Init)。
 */
void PID_Init(float Kp, float Ki, float Kd, float base_speed, float trim);

/**
 * @brief 完整 PID 更新周期 — 传感器 → 位置 → 滤波 → PID → 电机。
 *        Full PID update cycle: sensors → position → filter → PID → motors.
 * @param sensor_values 12路灰度传感器 bitmap (bit0=CH0, 1=白/亮, 0=黑/暗)
 *                      Grayscale_ReadAll() 的返回值
 * @return PID 输出修正量 (正值=左转, 负值=右转)
 *         PID output correction (positive = turn left, negative = turn right)
 */
float PID_Update(uint16_t sensor_values);

/*===========================================================================
 * 函数声明 — 分步 API (用于调试/自定义流程)
 *===========================================================================*/

/**
 * @brief 加权计算线位置。
 *        Weighted line position calculation from sensor bitmap.
 * @param sensor_values 12路灰度传感器 bitmap
 * @return 连续线位置 [-5.5, 5.5], 负=偏左, 正=偏右, 0=居中
 *         返回 0 且 is_lost=true 表示丢线 (全白/全黑)
 */
float PID_CalculatePosition(uint16_t sensor_values);

/**
 * @brief EMA 低通滤波 (指数移动平均)。
 *        Exponential Moving Average low-pass filter for position.
 * @param raw_position 原始加权位置
 * @return 滤波后的位置
 */
float PID_FilterPosition(float raw_position);

/**
 * @brief PID 核心计算。
 *        PID core computation with anti-windup and derivative filtering.
 * @param error 当前误差 (目标 - 实际)
 * @return PID 输出
 */
float PID_Compute(float error);

/*===========================================================================
 * 函数声明 — 状态查询 (调试/遥测)
 *===========================================================================*/

/**
 * @brief 获取滤波后的线位置 (用于串口调试)。
 *        Get filtered line position (for telemetry/debugging).
 */
float PID_GetFilteredPosition(void);

/**
 * @brief 获取当前 PID 输出修正量。
 *        Get current PID output correction value.
 */
float PID_GetOutput(void);

/**
 * @brief 获取当前误差值。
 *        Get current error value.
 */
float PID_GetError(void);

/**
 * @brief 获取积分项累加值。
 *        Get current integral term.
 */
float PID_GetIntegral(void);

/**
 * @brief 查询是否丢线 (全白/全黑, 无线索)。
 *        Check if line is lost (all sensors same = no line detected).
 * @return 0=线在, 1=丢线  0=line found, 1=line lost
 */
uint8_t PID_IsLineLost(void);

#endif /* __PID_H__ */
