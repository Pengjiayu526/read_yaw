/**
 * @file  corner.c
 * @brief 直角转弯判断模块实现 — 2态状态机: 循迹 ↔ 转弯
 *
 * 状态机:
 *   FOLLOW ──(全白检测)──▶ TURNING ──(转弯完成)──▶ FOLLOW
 *
 * 全白判定使用 PID_IsLineLost(), 3帧去抖防误触发。
 * 转弯方向由丢线前最后一帧的线位置决定:
 *   last_pos < 0 → 线偏左 → 左转 (-90°)
 *   last_pos > 0 → 线偏右 → 右转 (+90°)
 *
 * 非阻塞设计:
 *   - Corner_Task() 无 delay, 快速返回
 *   - 转弯期间 Turn_Task() 每轮驱动, GYRO_Poll() 在 Corner_Task 之前调用
 */

#include "corner.h"
#include "grayscale.h"
#include "pid.h"
#include "turn.h"

/*===========================================================================
 * 去抖参数
 *===========================================================================*/

/** @brief 全白持续多少帧后才确认直角 (防噪声误触发) */
#define CORNER_DEBOUNCE_FRAMES   3U

/*===========================================================================
 * 状态机定义
 *===========================================================================*/

typedef enum {
    STATE_FOLLOW  = 0,  /**< 正常循迹, PID 控制 */
    STATE_TURNING,       /**< 直角转弯中, Turn 状态机控制 */
} corner_state_t;

/*===========================================================================
 * 模块内部变量
 *===========================================================================*/

static corner_state_t gState       = STATE_FOLLOW;
static float          gLastPos     = 0.0f;   /**< 丢线前最后一帧线位置 */
static uint8_t        gLostCnt     = 0U;     /**< 全白连续帧计数 */
static uint8_t        gCornerCount = 0U;     /**< 已完成的直角转弯次数 */

/* PID 参数 — 转弯完成后重新初始化 PID 时使用 */
static float gKp, gKi, gKd, gBaseSpeed, gTrim;

/*===========================================================================
 * 公有函数
 *===========================================================================*/

void Corner_Init(float Kp, float Ki, float Kd, float base_speed, float trim)
{
    gKp = Kp;  gKi = Ki;  gKd = Kd;
    gBaseSpeed = base_speed;
    gTrim      = trim;

    PID_Init(Kp, Ki, Kd, base_speed, trim);
    Turn_Init();

    gState       = STATE_FOLLOW;
    gLastPos     = 0.0f;
    gLostCnt     = 0U;
    gCornerCount = 0U;
}

void Corner_Task(void)
{
    uint16_t sensors;

    switch (gState) {

    /*-----------------------------------------------------------------------
     * FOLLOW: PID 循迹 + 全白检测
     *---------------------------------------------------------------------*/
    case STATE_FOLLOW:
        sensors = Grayscale_ReadAll();
        PID_Update(sensors);

        if (PID_IsLineLost()) {
            /*
             * 全白: 累加去抖计数, 达阈值后触发转弯。
             * 转弯方向由丢线前最后一帧的线位置决定。
             */
            gLostCnt++;
            if (gLostCnt >= CORNER_DEBOUNCE_FRAMES) {
                int angle = (gLastPos < 0.0f) ? -90 : 90;
                Turn_Start(angle);
                gState   = STATE_TURNING;
                gLostCnt = 0U;
            }
        } else {
            /* 线在: 清零去抖计数, 更新最后有效位置 */
            gLostCnt = 0U;
            gLastPos = PID_GetFilteredPosition();
        }
        break;

    /*-----------------------------------------------------------------------
     * TURNING: 驱动转弯状态机, 完成后回到循迹
     *---------------------------------------------------------------------*/
    case STATE_TURNING:
        Turn_Task();
        if (!Turn_IsBusy()) {
            /* 转弯完成, 重新初始化 PID, 继续循迹 */
            PID_Init(gKp, gKi, gKd, gBaseSpeed, gTrim);
            gState = STATE_FOLLOW;
            gCornerCount++;
        }
        break;

    default:
        gState = STATE_FOLLOW;
        break;
    }
}

bool Corner_IsTurning(void)
{
    return (gState == STATE_TURNING);
}

uint8_t Corner_GetCount(void)
{
    return gCornerCount;
}
