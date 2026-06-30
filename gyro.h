#ifndef __GYRO_H
#define __GYRO_H

#include <stdint.h>

/**
 * @brief 陀螺仪数据结构体
 */
typedef struct {
    float    wz;           /**< Z轴角速度 (°/s) */
    float    yaw;          /**< 航向角 Yaw (°) */
    int16_t  raw_wz;       /**< Wz 原始ADC值 */
    int16_t  raw_yaw;      /**< Yaw 原始ADC值 */
    uint8_t  wz_updated;   /**< 收到新的 Wz 数据时置1 */
    uint8_t  yaw_updated;  /**< 收到新的 Yaw 数据时置1 */
} gyro_data_t;

/**
 * @brief UART 状态机状态定义
 */
typedef enum {
    GYRO_STATE_WAIT_HEADER = 0,  /**< 等待帧头 0x5A */
    GYRO_STATE_WAIT_TYPE,        /**< 等待类型字节 0xAA/0xBB */
    GYRO_STATE_WAIT_DL,          /**< 等待数据低字节 */
    GYRO_STATE_WAIT_DH,          /**< 等待数据高字节 */
    GYRO_STATE_WAIT_SUM          /**< 等待校验和 */
} gyro_state_t;

/* 陀螺仪协议常量 */
#define GYRO_HEADER          0x5A
#define GYRO_TYPE_ANGVEL      0xAA    /**< 角速度帧类型 */
#define GYRO_TYPE_YAW         0xBB    /**< 航向角帧类型 */
#define GYRO_WZ_SCALE        2000.0f  /**< Wz 量程 (°/s) */
#define GYRO_YAW_SCALE       180.0f   /**< Yaw 量程 (°) */
#define GYRO_SCALE_DIVISOR   32768.0f /**< int16 归一化因子 */

/* ---- 对外 API ---- */

/**
 * @brief 初始化陀螺仪解析模块
 * @note 调用前需确保 UART_2 已通过 SYSCFG_DL_init() 初始化
 */
void GYRO_Init(void);

/**
 * @brief 轮询处理 UART 接收缓冲区中的所有字节
 * @note 应在主循环中高频调用 (≥1kHz 以获得最佳实时性)
 */
void GYRO_Poll(void);

/**
 * @brief 获取最新的 Z 轴角速度
 * @return 角速度值 (°/s)
 */
float GYRO_GetWz(void);

/**
 * @brief 获取最新的航向角
 * @return 航向角 (°)
 */
float GYRO_GetYaw(void);

/**
 * @brief 是否收到新的 Wz 数据
 * @return 1 = 有新数据, 0 = 无
 */
uint8_t GYRO_IsWzUpdated(void);

/**
 * @brief 是否收到新的 Yaw 数据
 * @return 1 = 有新数据, 0 = 无
 */
uint8_t GYRO_IsYawUpdated(void);

/**
 * @brief 清除数据更新标志
 */
void GYRO_ClearUpdateFlags(void);

#endif /* __GYRO_H */
