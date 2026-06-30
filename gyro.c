/**
 * @file    gyro.c
 * @brief   M0G3507 MEMS 陀螺仪 UART 协议解析模块
 *
 * 协议帧格式 (来自 gyro_m0g3507_ti_uart.md):
 *   角速度: 0x5A  0xAA  AzL   AzH   SUM
 *   航向角: 0x5A  0xBB  YawL  YawH  SUM
 *
 *   数据解析: data = (int16_t)((DATAH << 8) | DATAL)
 *   物理量:   Wz  = data / 32768 * 2000  (°/s)
 *             Yaw = data / 32768 * 180   (°)
 *   校验:     SUM = (0x5A + TYPE + DL + DH) & 0xFF
 */

#include "gyro.h"
#include "ti_msp_dl_config.h"

/* ---- 模块内部变量 ---- */
static gyro_data_t  gyro_data;
static gyro_state_t state;
static uint8_t      frame_type;      /* 帧类型: 0xAA=角速度, 0xBB=航向角 */
static uint8_t      frame_dl;        /* 数据低字节 */
static uint8_t      frame_dh;        /* 数据高字节 */

/* ---- 内部辅助函数 ---- */

/**
 * @brief 检查 UART RX FIFO 中是否有数据可读
 */
static inline uint8_t gyro_byte_available(void)
{
    return !DL_UART_Main_isRXFIFOEmpty(UART_2_INST);
}

/**
 * @brief 从 UART RX FIFO 读取一个字节 (调用前请确保有数据)
 */
static inline uint8_t gyro_read_byte(void)
{
    return DL_UART_Main_receiveData(UART_2_INST);
}

/**
 * @brief 将收到的单个字节送入状态机处理
 */
static void gyro_process_byte(uint8_t byte)
{
    switch (state) {

    case GYRO_STATE_WAIT_HEADER:
        if (byte == GYRO_HEADER) {
            state = GYRO_STATE_WAIT_TYPE;
        }
        break;

    case GYRO_STATE_WAIT_TYPE:
        if (byte == GYRO_TYPE_ANGVEL || byte == GYRO_TYPE_YAW) {
            frame_type = byte;
            state      = GYRO_STATE_WAIT_DL;
        } else if (byte == GYRO_HEADER) {
            /* 收到意外的帧头, 重新开始等待类型 */
            state = GYRO_STATE_WAIT_TYPE;
        } else {
            /* 无效的类型字节, 回退到等待帧头 */
            state = GYRO_STATE_WAIT_HEADER;
        }
        break;

    case GYRO_STATE_WAIT_DL:
        frame_dl = byte;
        state    = GYRO_STATE_WAIT_DH;
        break;

    case GYRO_STATE_WAIT_DH:
        frame_dh = byte;
        state    = GYRO_STATE_WAIT_SUM;
        break;

    case GYRO_STATE_WAIT_SUM:
        {
            /* 校验和 = (帧头 + 类型 + 数据低 + 数据高) 低8位 */
            uint8_t calc_sum =
                (GYRO_HEADER + frame_type + frame_dl + frame_dh) & 0xFF;

            if (byte == calc_sum) {
                /* 校验通过, 解析数据 */
                int16_t raw =
                    (int16_t)(((uint16_t)frame_dh << 8) | (uint16_t)frame_dl);

                if (frame_type == GYRO_TYPE_ANGVEL) {
                    gyro_data.raw_wz = raw;
                    gyro_data.wz =
                        (float)raw / GYRO_SCALE_DIVISOR * GYRO_WZ_SCALE;
                    gyro_data.wz_updated = 1;
                } else { /* GYRO_TYPE_YAW */
                    gyro_data.raw_yaw = raw;
                    gyro_data.yaw =
                        (float)raw / GYRO_SCALE_DIVISOR * GYRO_YAW_SCALE;
                    gyro_data.yaw_updated = 1;
                }
            }
            /* 无论校验是否通过, 都回到等待下一帧的帧头 */
            state = GYRO_STATE_WAIT_HEADER;
        }
        break;

    default:
        state = GYRO_STATE_WAIT_HEADER;
        break;
    }
}

/* ---- 对外 API ---- */

void GYRO_Init(void)
{
    state = GYRO_STATE_WAIT_HEADER;

    gyro_data.wz          = 0.0f;
    gyro_data.yaw         = 0.0f;
    gyro_data.raw_wz      = 0;
    gyro_data.raw_yaw     = 0;
    gyro_data.wz_updated  = 0;
    gyro_data.yaw_updated = 0;
}

void GYRO_Poll(void)
{
    while (gyro_byte_available()) {
        gyro_process_byte(gyro_read_byte());
    }
}

float   GYRO_GetWz(void)        { return gyro_data.wz; }
float   GYRO_GetYaw(void)       { return gyro_data.yaw; }
uint8_t GYRO_IsWzUpdated(void)  { return gyro_data.wz_updated; }
uint8_t GYRO_IsYawUpdated(void) { return gyro_data.yaw_updated; }

void GYRO_ClearUpdateFlags(void)
{
    gyro_data.wz_updated  = 0;
    gyro_data.yaw_updated = 0;
}
