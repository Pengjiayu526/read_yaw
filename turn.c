/**
 * @file    turn.c
 * @brief   两轮差速小车原地直角转弯模块实现
 *
 * 核心设计:
 *   - 状态机驱动: IDLE → SPIN → STOP → IDLE
 *   - P 控制器闭环: 陀螺仪 Yaw 角 → 角度误差 → P控制 → PWM 占空比
 *   - 角度跳变处理: ±180° 归一化到最短路径
 *   - 无延时阻塞: Turn_Task() 内部不使用 delay, 由主循环调频
 *
 * 数据流:
 *   GYRO_Poll() → GYRO_GetYaw() → 角度误差 → 控制器 → Motor_SetSpeed()
 *
 * 差速转弯策略 (无反转模式的原地 pivot turn):
 *   - 右转: 左轮前进, 右轮停止 → 车体绕右轮顺时针旋转
 *   - 左转: 右轮前进, 左轮停止 → 车体绕左轮逆时针旋转
 *
 * 停止条件 (双重判定):
 *   1. |角度误差| < 2°   (已到达目标角度)
 *   2. |Z轴角速度| < 5°/s (车体已停稳, 无振荡)
 */

#include "turn.h"
#include "motor.h"
#include "gyro.h"

/*===========================================================================
 * 状态机定义
 *===========================================================================*/

typedef enum {
    TURN_STATE_IDLE = 0,  /**< 空闲, 等待指令 */
    TURN_STATE_SPIN,      /**< 旋转中, 闭环控制 */
    TURN_STATE_STOP,      /**< 刹车过渡态 */
} turn_state_t;

/*===========================================================================
 * 控制参数 (可调)
 *===========================================================================*/

/**
 * @brief P 控制器比例系数。
 * @note  选取依据: 初始误差 90° × 0.6 = 54% 占空比, 处于 40~60% 建议区间。
 *        调大 → 响应快但易超调; 调小 → 平缓但可能转不动。
 */
#define TURN_KP          0.6f

/** @brief 最小 PWM 占空比 (%), 避免输出过弱导致电机堵转 */
#define TURN_MIN_SPEED   25

/** @brief 最大 PWM 占空比 (%), 保护电机不过流 */
#define TURN_MAX_SPEED   60

/** @brief 角度误差阈值 (°), 低于此值判定到位 */
#define TURN_ERR_THRESH  2.0f

/** @brief 角速度阈值 (°/s), 低于此值判定停稳 */
#define TURN_WZ_THRESH   5.0f

/*===========================================================================
 * 模块内部变量
 *===========================================================================*/

static turn_state_t turn_state      = TURN_STATE_IDLE;
static float        turn_target_yaw;  /**< 目标绝对 Yaw 角 (已归一化到 ±180) */
static int          turn_direction;   /**< +1 = 右转 (顺时针), -1 = 左转 (逆时针) */

/*===========================================================================
 * 内部辅助函数
 *===========================================================================*/

/**
 * @brief 计算从 current 到 target 的最短角度误差。
 * @param target  目标角度 (°)
 * @param current 当前角度 (°)
 * @return 误差角 (°), 正值 = 需向右转, 负值 = 需向左转
 *
 * 处理 ±180° 跳变:
 *   例如 target=170°, current=-170° → 原始差 340° → 归一化后 -20° (左转 20°)
 *   例如 target=-170°, current=170° → 原始差 -340° → 归一化后 +20° (右转 20°)
 */
static float angle_error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f) {
        err -= 360.0f;
    }
    while (err < -180.0f) {
        err += 360.0f;
    }
    return err;
}

/**
 * @brief 将 value 限制在 [min, max] 闭区间内。
 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 浮点数取绝对值 (避免链接 math.h)。
 */
static float fabsf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

/*===========================================================================
 * 公有函数
 *===========================================================================*/

void Turn_Init(void)
{
    turn_state      = TURN_STATE_IDLE;
    turn_direction  = 0;
    turn_target_yaw = 0.0f;
}

void Turn_Start(int angle)
{
    /* 忙时忽略新指令, 避免状态混乱 */
    if (turn_state != TURN_STATE_IDLE) {
        return;
    }

    float start_yaw = GYRO_GetYaw();

    /* 计算目标绝对 Yaw 角 */
    turn_target_yaw = start_yaw + (float)angle;

    /* 归一化到 [-180, +180] 区间 */
    while (turn_target_yaw > 180.0f) {
        turn_target_yaw -= 360.0f;
    }
    while (turn_target_yaw < -180.0f) {
        turn_target_yaw += 360.0f;
    }

    turn_direction = (angle >= 0) ? 1 : -1;
    turn_state     = TURN_STATE_SPIN;
}

void Turn_Task(void)
{
    float error;
    float output;

    switch (turn_state) {

    case TURN_STATE_IDLE:
        /* 空闲, 不做任何操作 */
        break;

    case TURN_STATE_SPIN:
    {
        /* ---- 获取当前姿态 ---- */
        float current_yaw = GYRO_GetYaw();
        float current_wz  = GYRO_GetWz();

        /* ---- 计算角度误差 (已处理 ±180° 跳变) ---- */
        error = angle_error(turn_target_yaw, current_yaw);

        float abs_err = fabsf_local(error);
        float abs_wz  = fabsf_local(current_wz);

        /* ---- 停止条件: 误差小 且 已停稳 ---- */
        if (abs_err < TURN_ERR_THRESH && abs_wz < TURN_WZ_THRESH) {
            Motor_Stop();
            turn_state = TURN_STATE_STOP;
            break;
        }

        /* ---- P 控制: output = Kp × |error| ---- */
        output = TURN_KP * abs_err;

        /* 限制速度范围, 防止堵转或过流 */
        output = clampf(output, (float)TURN_MIN_SPEED, (float)TURN_MAX_SPEED);

        /*
         * 差速驱动 — pivot turn (单轮前进, 另一轮静止):
         *   右转 (+1): 左轮前进 → 车体绕右轮顺时针旋转
         *   左转 (-1): 右轮前进 → 车体绕左轮逆时针旋转
         */
        uint8_t left_speed, right_speed;
        if (turn_direction > 0) {
            left_speed  = (uint8_t)output;
            right_speed = 0;
        } else {
            left_speed  = 0;
            right_speed = (uint8_t)output;
        }

        Motor_SetSpeed(left_speed, right_speed);
        break;
    }

    case TURN_STATE_STOP:
        /* 电机已在进入 STOP 时制动, 此处仅清理状态回到空闲 */
        turn_state     = TURN_STATE_IDLE;
        turn_direction = 0;
        break;

    default:
        /* 异常状态回退, 安全起见停止电机 */
        Motor_Stop();
        turn_state = TURN_STATE_IDLE;
        break;
    }
}

bool Turn_IsBusy(void)
{
    return (turn_state != TURN_STATE_IDLE);
}
