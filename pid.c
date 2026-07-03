/**
 * @file  pid.c
 * @brief PID 循迹控制器实现 — 加权位置 + EMA双滤波 + PID核心 + 差速转向
 *        PID line-following controller with weighted position calculation,
 *        dual EMA filtering, anti-windup PID, and differential steering.
 *
 * 算法细节:
 * ┌──────────┐    ┌──────────┐    ┌─────────┐    ┌──────────┐    ┌────────┐
 * │ 12路灰度 │ → │ 加权位置  │ → │ EMA滤波 │ → │ PID计算  │ → │ 差速电机│
 * │ 传感器   │    │ 计算      │    │ (位置)  │    │ P+I+D    │    │ 输出   │
 * └──────────┘    └──────────┘    └─────────┘    └──────────┘    └────────┘
 *                                                  │
 *                                          ┌───────┴───────┐
 *                                          │ 抗积分饱和    │
 *                                          │ 微分项EMA滤波 │
 *                                          └───────────────┘
 *
 * 加权方法 (Weighted Position):
 *   12个传感器等距排列, 中心在传感器5和6之间。
 *   权重数组: [-5.5, -4.5, ..., -0.5, 0.5, ..., 5.5]
 *   线位置 = Σ(权重[i] × 线检测[i]) / Σ(线检测[i])
 *   其中 线检测[i] = (传感器读数为1即黑线) ? 1 : 0
 *
 * 滤波策略 (Filtering Strategy):
 *   1. 位置 EMA 滤波: 抑制传感器离散化噪声和抖动
 *      filtered_pos = α × raw + (1-α) × prev_filtered
 *   2. 微分项 EMA 滤波: 抑制高频噪声被微分放大
 *      deriv_filtered = α_d × deriv_raw + (1-α_d) × prev_deriv
 *
 * PID 公式 (discrete-time, dt 已吸收至 Ki/Kd):
 *   error   = setpoint - filtered_position
 *   P_out   = Kp × error
 *   I_sum  += Ki × error      (条件积分: 仅在未饱和时累加)
 *   D_raw   = Kd × (error - prev_error)
 *   D_out   = α_d × D_raw + (1-α_d) × D_out_prev   (EMA滤波)
 *   output  = P_out + I_sum + D_out
 *   output  = clamp(output, ±output_limit)
 *
 * 差速转向 (Differential Steering):
 *   左轮速度 = base_speed - output + trim  (正trim → 左轮多出力 → 右转倾向)
 *   右轮速度 = base_speed + output - trim
 *   trim 补偿物理偏斜, 速度均钳位在 [0, 100] 范围内
 */

#include "pid.h"
#include "motor.h"

/*===========================================================================
 * 传感器权重表 — 物理位置对称分布
 * Sensor weight lookup table — symmetric physical positions
 *
 * 传感器排列示意 (俯视图, 车头朝上):
 *   CH0  CH1  CH2  CH3  CH4  CH5  |  CH6  CH7  CH8  CH9  CH10 CH11
 *   左←←←←←←←←←←←←←←← 中心 →→→→→→→→→→→→→→→→→→→右
 *   -5.5 -4.5 -3.5 -2.5 -1.5 -0.5 | +0.5 +1.5 +2.5 +3.5 +4.5 +5.5
 *===========================================================================*/
static const float gWeights[PID_SENSOR_COUNT] = {
    -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f,  5.5f
};

/*===========================================================================
 * PID 参数 — 由 PID_Init() 设定
 * PID parameters — set by PID_Init()
 *===========================================================================*/
static float gKp = 1.0f;             /* 比例增益 Proportional gain        */
static float gKi = 0.0f;             /* 积分增益 Integral gain            */
static float gKd = 0.5f;             /* 微分增益 Derivative gain          */
static float gSetpoint = 0.0f;       /* 目标位置 (0=居中) Target position */

/*===========================================================================
 * 限幅参数
 * Limit / clamping parameters
 *===========================================================================*/
static float gOutputLimit   = 50.0f; /* 输出限幅 (跟随 base_speed)        */
static float gIntegralLimit = 30.0f; /* 积分限幅 (抗饱和 anti-windup)     */
static float gBaseSpeed     = 50.0f; /* 基础电机速度 Base motor speed     */
static float gTrim          = 0.0f;  /* 左右轮补偿 Trim for L/R imbalance */

/*===========================================================================
 * EMA 滤波系数 (指数移动平均)
 * EMA filter coefficients
 *===========================================================================*/
static float gPosAlpha   = 0.2f;     /* 位置滤波系数 (0~1, 小=强滤波)    */
static float gDerivAlpha = 0.1f;     /* 微分滤波系数 (0~1, 小=强滤波)    */

/*===========================================================================
 * PID 运行时状态 — 由 PID_Init() 初始化
 * PID runtime state — initialized by PID_Init()
 *===========================================================================*/
static float gError        = 0.0f;   /* 当前误差 current error            */
static float gPrevError    = 0.0f;   /* 上次误差 previous error (微分用)  */
static float gIntegral     = 0.0f;   /* 积分累加 integral accumulator     */
static float gDerivative   = 0.0f;   /* 滤波后微分 filtered derivative    */
static float gOutput       = 0.0f;   /* PID 最终输出 final output         */

/*===========================================================================
 * 滤波器状态
 * Filter state
 *===========================================================================*/
static float   gFilteredPos = 0.0f;   /* 滤波后位置 filtered position      */
static uint8_t gFilterInit  = 1U;     /* 首次初始化标志 first-init flag    */

/*===========================================================================
 * 丢线处理
 * Line-lost handling
 *===========================================================================*/
static float  gLastValidPos = 0.0f;  /* 最后有效位置 last valid position  */
static uint8_t gLineLost    = 0U;    /* 丢线标志 line-lost flag           */
static uint8_t gPrevLineLost = 0U;   /* 上一周期丢线标志 prev lost flag   */

/*===========================================================================
 * 加权位置计算
 * Weighted position calculation
 *===========================================================================*/
float PID_CalculatePosition(uint16_t sensor_values)
{
    float weighted_sum = 0.0f;
    uint8_t active_count = 0;

    /*
     * 遍历12个传感器通道:
     * - 传感器读数 = 1 → 黑线 (检测到黑线) → line_value = 1
     * - 传感器读数 = 0 → 白底 (无线索)    → line_value = 0
     */
    for (uint8_t i = 0; i < PID_SENSOR_COUNT; i++) {
        uint8_t bit_val = (sensor_values >> i) & 0x01U;
        uint8_t line_detected = (bit_val == 1U) ? 1U : 0U;

        if (line_detected) {
            weighted_sum += gWeights[i];
            active_count++;
        }
    }

    /*
     * 处理边缘情况:
     * - 所有传感器读白 (active_count == 0): 丢线, 保持上次有效位置
     * - 所有传感器读黑 (active_count == 12): 全部在线, 判为居中
     */
    if (active_count == 0) {
        /* 丢线 — 保持最后有效位置, 设标志 */
        gLineLost = 1U;
        return gLastValidPos;
    }

    /* 正常情况: 加权平均 */
    gLineLost = 0U;
    float position = weighted_sum / (float)active_count;
    gLastValidPos = position;
    return position;
}

/*===========================================================================
 * EMA 低通滤波
 * EMA low-pass filter for position
 *
 * 公式: y[n] = α × x[n] + (1-α) × y[n-1]
 *   α → 1: 几乎不过滤 (响应快, 噪声大)
 *   α → 0: 强滤波 (响应慢, 平滑)
 *===========================================================================*/
float PID_FilterPosition(float raw_position)
{
    if (gFilterInit) {
        /* 首次调用: 直接使用原始值, 避免从0缓慢上升 */
        gFilteredPos = raw_position;
        gFilterInit = 0U;
    } else {
        gFilteredPos = gPosAlpha * raw_position
                     + (1.0f - gPosAlpha) * gFilteredPos;
    }
    return gFilteredPos;
}

/*===========================================================================
 * PID 核心计算 (含抗积分饱和 + 微分EMA滤波)
 * PID core computation with anti-windup and derivative EMA filtering
 *===========================================================================*/
float PID_Compute(float error)
{
    float p_out, d_raw, i_term;
    float output_unclamped;

    /* ---- 比例项 P ---- */
    p_out = gKp * error;

    /* ---- 微分项 D (带 EMA 滤波) ----
     * 原始微分 = Kd × Δerror (dt 已吸收至 Kd)
     * 滤波微分 = α_d × 原始微分 + (1-α_d) × 上次滤波微分
     */
    d_raw = gKd * (error - gPrevError);
    gDerivative = gDerivAlpha * d_raw
                + (1.0f - gDerivAlpha) * gDerivative;

    /* ---- 积分项 I (条件积分 — 抗饱和) ----
     * 先计算未钳位的输出, 若超限则跳过本次积分累加,
     * 避免积分在输出饱和时继续增长 (积分饱和 / windup)
     */
    i_term = gIntegral + gKi * error;
    output_unclamped = p_out + i_term + gDerivative;

    if (output_unclamped > gOutputLimit) {
        /* 正饱和: 不累加积分 (如果积分在正方向增长) */
        if (gKi * error > 0.0f) {
            /* 保持原积分值 */;
        } else {
            gIntegral = i_term;
        }
        gOutput = gOutputLimit;
    } else if (output_unclamped < -gOutputLimit) {
        /* 负饱和: 不累加积分 (如果积分在负方向增长) */
        if (gKi * error < 0.0f) {
            /* 保持原积分值 */;
        } else {
            gIntegral = i_term;
        }
        gOutput = -gOutputLimit;
    } else {
        /* 未饱和: 正常累加积分 */
        gIntegral = i_term;
        gOutput = output_unclamped;
    }

    /* 积分额外限幅 (防止长期累积超过合理范围) */
    if (gIntegral > gIntegralLimit) {
        gIntegral = gIntegralLimit;
    } else if (gIntegral < -gIntegralLimit) {
        gIntegral = -gIntegralLimit;
    }

    /* 保存本次误差, 供下次微分计算 */
    gPrevError = error;

    return gOutput;
}

/*===========================================================================
 * 完整 PID 更新周期
 * Full PID update cycle
 *===========================================================================*/
float PID_Update(uint16_t sensor_values)
{
    float raw_position, filtered_position, error;
    float output;
    float left_speed, right_speed;

    /* Step 1: 加权位置计算 */
    raw_position = PID_CalculatePosition(sensor_values);

    /* Step 1.5: 丢线检查 — 全白时停车 */
    if (gLineLost) {
        Motor_Stop();
        gOutput = 0.0f;
        gPrevLineLost = 1U;
        return 0.0f;
    }

    /* 重新找到线 — 清空积分和滤波状态, 避免上次运行的残留 */
    if (gPrevLineLost) {
        gIntegral    = 0.0f;
        gDerivative  = 0.0f;
        gPrevError   = 0.0f;
        gFilterInit  = 1U;
        gPrevLineLost = 0U;
    }

    /* Step 2: EMA 低通滤波 */
    filtered_position = PID_FilterPosition(raw_position);

    /* Step 3: 计算误差 (目标 - 实际)
     * setpoint 默认为 0 (居中), 正误差=线偏左, 负误差=线偏右 */
    gError = gSetpoint - filtered_position;

    /* Step 4: PID 计算 */
    output = PID_Compute(gError);

    /* Step 5: 差速转向 — 将 PID 输出分配到左右电机
     * 左轮 = base - output + trim (正trim → 左轮多出力 → 右转倾向)
     * 右轮 = base + output - trim
     * trim 补偿物理偏斜: 车老偏左设正值, 老偏右设负值 */
    left_speed  = gBaseSpeed - output + gTrim;
    right_speed = gBaseSpeed + output - gTrim;

    /* 速度钳位 [0, 100] */
    if (left_speed < 0.0f)  left_speed = 0.0f;
    if (left_speed > 100.0f) left_speed = 100.0f;
    if (right_speed < 0.0f)  right_speed = 0.0f;
    if (right_speed > 100.0f) right_speed = 100.0f;

    /* Step 6: 更新电机 */
    Motor_SetSpeed((uint8_t)left_speed, (uint8_t)right_speed);

    return output;
}

/*===========================================================================
 * 初始化
 * Initialization
 *===========================================================================*/
void PID_Init(float Kp, float Ki, float Kd, float base_speed, float trim)
{
    gKp = Kp;
    gKi = Ki;
    gKd = Kd;
    gSetpoint = 0.0f;

    /* 输出限幅设为与基础速度相同 (修正量不超过基础速度) */
    gBaseSpeed     = base_speed;
    gTrim          = trim;
    gOutputLimit   = base_speed;
    gIntegralLimit = base_speed * 0.3f;  /* 积分上限 = 30% base_speed */

    /* 初始化所有运行时状态 */
    gError       = 0.0f;
    gPrevError   = 0.0f;
    gIntegral    = 0.0f;
    gDerivative  = 0.0f;
    gOutput      = 0.0f;
    gFilteredPos = 0.0f;
    gFilterInit  = 1U;
    gLastValidPos = 0.0f;
    gLineLost     = 0U;
    gPrevLineLost = 0U;
}

/*===========================================================================
 * 状态查询 (调试/遥测)
 * State query (debugging / telemetry)
 *===========================================================================*/
float PID_GetFilteredPosition(void) { return gFilteredPos; }
float PID_GetOutput(void)           { return gOutput;       }
float PID_GetError(void)            { return gError;        }
float PID_GetIntegral(void)         { return gIntegral;     }
uint8_t PID_IsLineLost(void)        { return gLineLost; }
