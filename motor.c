/**
 * @file  motor.c
 * @brief 双轮电机驱动实现 — 通过改变 PWM 波占空比控制转速
 *
 * 硬件参数 (由 SysConfig 生成):
 * - TIMG0 定时器, PWM 时钟 = 1.28 MHz (32MHz / 25)
 * - PWM 周期 = 128 个时钟周期 → PWM 频率 = 10000 Hz
 * - 右轮: TIMG0 通道0 (CCP0, PA12)
 * - 左轮: TIMG0 通道1 (CCP1, PA13)
 *
 * 速度控制原理:
 *   PWM 占空比 = 比较值 / 周期
 *   比较值 0   →   0% 占空比 → 电机停转
 *   比较值 128 → 100% 占空比 → 电机全速
 */

#include "motor.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 硬件映射
 *===========================================================================*/

/** @brief PWM 定时器实例 */
#define MOTOR_PWM           PWM_0_INST           /* TIMG0 */

/** @brief 右轮 CC 索引 (通道0) */
#define MOTOR_CC_RIGHT      DL_TIMER_CC_0_INDEX

/** @brief 左轮 CC 索引 (通道1) */
#define MOTOR_CC_LEFT       DL_TIMER_CC_1_INDEX

/*===========================================================================
 * 内部函数
 *===========================================================================*/

/**
 * @brief 将速度百分比 (0~100) 转换为 PWM 比较值。
 * @note  比较值与占空比成反比: 比较值=0→100%, 比较值=127→0%
 *        计数器 LOAD = period-1 = 127, 比较值必须 ≤127, 写128永不匹配→全速
 * @param speed 速度百分比, 超出 100 截断为 100
 * @return 对应的 PWM 比较寄存器值 (0 ~ 127)
 */
static uint32_t Motor_SpeedToCompare(uint8_t speed)
{
    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    }
    /* cmp = (period-1) * (1 - speed/100), 有效范围 0~127 */
    return (MOTOR_PWM_PERIOD - 1)
           - ((uint32_t)speed * (MOTOR_PWM_PERIOD - 1)) / MOTOR_SPEED_MAX;
}

/*===========================================================================
 * 公有函数
 *===========================================================================*/

 //电机驱动板的所有线都要连接到主控板上去
void Motor_Init(void)
{
    /*
     * 先将两路比较值设为最大值 (period-1=127), 确保定时器启动时
     * 占空比 ≈ 0%, 电机保持停止, 等待后续 Motor_SetSpeed() 设定速度。
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_LEFT);

    /* 启动定时器计数, PWM 波形开始输出 */
    DL_TimerG_startCounter(MOTOR_PWM);
}

void Motor_SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed)
{
    uint32_t leftCompare  = Motor_SpeedToCompare(leftSpeed);
    uint32_t rightCompare = Motor_SpeedToCompare(rightSpeed);

    /*
     * 更新两路 PWM 的比较值以改变占空比:
     * - 比较值 = 0:   输出始终为低, 电机两端电压为 0, 停止
     * - 比较值 = 128: 输出始终为高, 电机两端电压 = 电源电压, 全速
     * - 中间值:       输出方波, 占空比 = 比较值/128, 电机转速与占空比成正比
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM, rightCompare, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM, leftCompare,  MOTOR_CC_LEFT);
}

void Motor_Stop(void)
{
    /*
     * 比较值 = period-1 = 127 → 占空比 ≈ 0% → 电机停止
     * 注意: 不能写 128, 因为计数器只到 127, 写 128 永不匹配反而 100% 全速
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_LEFT);
}
