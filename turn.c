/**
 * @file    turn.c
 * @brief   两轮差速小车原地直角转弯模块实现
 *
 * 核心设计:
 *   - 状态机驱动: IDLE → SPIN → STOP → IDLE
 *   - PD 控制器闭环: 角度误差(P项) + 角速度抑制(D项) → PWM 占空比
 *   - 角度跳变处理: ±180° 归一化到最短路径
 *   - 超时保护: SPIN 超过时限强制结束, 防止永久卡死
 *   - 无延时阻塞: Turn_Task() 内部不使用 delay, 由主循环调频
 *
 * 数据流:
 *   GYRO_Poll() → GYRO_GetYaw() → 角度误差 → PD控制器 → Motor_SetSpeed()
 *                              GYRO_GetWz()  → 角速度抑制(D项) ↗
 *
 * PD 控制器:
 *   pd_out = Kp × error - Kd × Wz
 *   - P项: 远离目标 → 加速; 靠近目标 → 自然减速
 *   - D项: 转速越快刹车越强, pd_out 变号时主动驱动对侧轮刹车
 *
 * 最小驱动力:
 *   始终保证 abs_out ≥ 6%, 防止 PD 输出太弱导致电机推不动静摩擦。
 *   6% 足够低, 接近目标时不会造成大幅过冲, D 项可以轻松刹住。
 */

#include "turn.h"
#include "motor.h"
#include "gyro.h"

/*===========================================================================
 * 状态机定义
 *===========================================================================*/

typedef enum {
    TURN_STATE_IDLE = 0,  /**< 空闲, 等待指令 */
    TURN_STATE_SPIN,      /**< 旋转中, PD 闭环控制 */
    TURN_STATE_STOP,      /**< 刹车 → 回到空闲的过渡态 */
} turn_state_t;

/*===========================================================================
 * 控制参数 (可调)
 *===========================================================================*/

/** @brief P 项比例系数 (角度误差 → 驱动力) */
#define TURN_KP          0.70f

/** @brief D 项微分系数 (角速度 → 刹车力), Kd/Kp=0.26 */
#define TURN_KD          0.18f

/** @brief 最小驱动力 (%), 防止 PD 输出太低推不动静摩擦 */
#define TURN_MIN_DRIVE   6

/** @brief 最大 PWM 占空比 (%) */
#define TURN_MAX_SPEED   60

/** @brief 角度误差阈值 (°), 低于此值认为角度到位 */
#define TURN_ERR_THRESH  2.5f

/** @brief 角速度阈值 (°/s), 低于此值认为已停稳 */
#define TURN_WZ_THRESH   6.0f

/**
 * @brief SPIN 超时迭代次数。
 * @note  主循环约 10~50 us/次, 500000 次 ≈ 5~25 秒。
 *        超时后强制结束转弯, 防止因传感器异常或机械卡死导致永久阻塞。
 */
/** @brief SPIN 超时迭代次数 (约 2~10 秒) */
#define TURN_TIMEOUT_TICKS   200000UL

/** @brief 误差连续达标多少次后强制结束 (约 0.5~2 秒, 抗 Wz 偏置) */
#define TURN_SETTLE_TICKS     50000UL

/*===========================================================================
 * 模块内部变量
 *===========================================================================*/

static turn_state_t turn_state      = TURN_STATE_IDLE;
static float        turn_target_yaw;  /**< 目标绝对 Yaw 角 (已归一化到 ±180) */
static uint32_t     spin_ticks;       /**< SPIN 状态累计迭代次数 (超时用) */
static uint32_t     settle_cnt;       /**< 连续误差达标次数 (抗 Wz 偏置卡死) */

/*===========================================================================
 * 内部辅助函数
 *===========================================================================*/

/**
 * @brief 计算从 current 到 target 的最短角度误差。
 * @return 正值 = 需向右转, 负值 = 需向左转
 */
static float angle_error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f)  { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }
    return err;
}

/** @brief 浮点数取绝对值 */
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
    turn_target_yaw = 0.0f;
    spin_ticks      = 0;
    settle_cnt      = 0;
}

void Turn_Start(int angle)
{
    if (turn_state != TURN_STATE_IDLE) {
        return;  /* 忙时忽略 */
    }

    float start_yaw = GYRO_GetYaw();

    /* 计算目标绝对 Yaw 角并归一化到 [-180, +180] */
    turn_target_yaw = start_yaw + (float)angle;
    while (turn_target_yaw > 180.0f)  { turn_target_yaw -= 360.0f; }
    while (turn_target_yaw < -180.0f) { turn_target_yaw += 360.0f; }

    spin_ticks  = 0;
    settle_cnt  = 0;
    turn_state  = TURN_STATE_SPIN;
}

void Turn_Task(void)
{
    switch (turn_state) {

    case TURN_STATE_IDLE:
        break;

    case TURN_STATE_SPIN:
    {
        float current_yaw = GYRO_GetYaw();
        float current_wz  = GYRO_GetWz();
        float error       = angle_error(turn_target_yaw, current_yaw);
        float abs_err     = fabsf_local(error);
        float abs_wz      = fabsf_local(current_wz);

        /* ---- 超时保护: 强制结束, 防止永久卡死 ---- */
        spin_ticks++;
        if (spin_ticks > TURN_TIMEOUT_TICKS) {
            Motor_Stop();
            turn_state = TURN_STATE_STOP;
            break;
        }

        /* ---- 停止判定 (解耦误差和 Wz) ---- */
        if (abs_err < TURN_ERR_THRESH) {
            /*
             * 误差已达标。关电机, 累加 settle 计数。
             *
             * 两种情况会触发完成:
             *   1. abs_wz < WZ_THRESH → 车已停稳, 立即结束
             *   2. settle_cnt 超限   → Wz 偏置/噪声导致永远不达标,
             *                          但误差持续在范围内, 强制结束
             *
             * 如果中途误差漂出阈值, settle_cnt 清零, 重新 PD 控制。
             */
            Motor_Stop();
            settle_cnt++;

            if (abs_wz < TURN_WZ_THRESH || settle_cnt > TURN_SETTLE_TICKS) {
                turn_state = TURN_STATE_STOP;
            }
            break;
        }

        /* 误差未达标, 清零 settle 计数, 继续 PD 控制 */
        settle_cnt = 0;

        /* ---- PD 控制器 ---- */
        float pd_out  = TURN_KP * error - TURN_KD * current_wz;
        float abs_out = fabsf_local(pd_out);

        /* 上限钳位 */
        if (abs_out > (float)TURN_MAX_SPEED) {
            abs_out = (float)TURN_MAX_SPEED;
        }

        /*
         * 最小驱动力: 始终保证输出 ≥ MIN_DRIVE。
         * 避免 PD 输出 1~5% 推不动静摩擦导致车卡住不到位。
         * MIN_DRIVE=6 足够低, D 项可以轻松刹车, 不会造成大振幅振荡。
         */
        if (abs_out < (float)TURN_MIN_DRIVE) {
            abs_out = (float)TURN_MIN_DRIVE;
        }

        /* pd_out 符号决定转向, abs_out 决定力度 */
        if (pd_out > 0.0f) {
            Motor_SetSpeed((uint8_t)abs_out, 0);   /* 右转: 左轮前进 */
        } else {
            Motor_SetSpeed(0, (uint8_t)abs_out);   /* 左转: 右轮前进 */
        }
        break;
    }

    case TURN_STATE_STOP:
        Motor_Stop();
        turn_state = TURN_STATE_IDLE;
        break;

    default:
        Motor_Stop();
        turn_state = TURN_STATE_IDLE;
        break;
    }
}

bool Turn_IsBusy(void)
{
    return (turn_state != TURN_STATE_IDLE);
}
