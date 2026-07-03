/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "gyro.h"
#include "motor.h"
#include "corner.h"
#include "delay.h"

/* ---- 显示参数 ---- */
#define FONT_SIZE        16          /* 使用16像素字体 */
#define DISPLAY_INTERVAL 100         /* 显示刷新间隔 (ms) */

/*
 * 主循环空闲迭代 ≈ 2~5 us/次。
 * 50,000 次 × 4 us ≈ 200 ms 用于显示刷新。
 */
#define DISPLAY_LOOP_CNT   50000    /* 显示刷新迭代间隔 */

/* ---- PID 参数 (可调) ---- */
#define CORNER_KP          1.0f    /* 比例增益 */
#define CORNER_KI          0.0f    /* 积分增益 (直角循迹建议先关积分) */
#define CORNER_KD          0.5f    /* 微分增益 */
#define CORNER_BASE_SPEED  45.0f   /* 基础速度 (%) */
#define CORNER_TRIM        0.0f    /* 左右补偿 (正值=右转倾向) */

/* ---- 内部辅助函数 ---- */

/**
 * @brief 在 OLED 指定位置显示带符号的浮点数
 */
static void OLED_ShowSignedFloat(u8 x, u8 y, float val,
                                  u8 int_w, u8 frac_w, u8 size)
{
    u8 x_pos = x;
    u8 char_w = size / 2;

    if (val < 0.0f) {
        OLED_ShowChar(x_pos, y, '-', size);
        val = -val;
    } else {
        OLED_ShowChar(x_pos, y, ' ', size);
    }
    x_pos += char_w;

    u32 int_part = (u32)val;
    OLED_ShowNum(x_pos, y, int_part, int_w, size);
    x_pos += int_w * char_w;

    OLED_ShowChar(x_pos, y, '.', size);
    x_pos += char_w;

    float frac = val - (float)int_part;
    u32 mult = 1;
    for (u8 i = 0; i < frac_w; i++) mult *= 10;
    u32 frac_part = (u32)(frac * (float)mult + 0.5f);
    if (frac_part >= mult) frac_part = 0;

    OLED_ShowNum(x_pos, y, frac_part, frac_w, size);
}

/**
 * @brief 刷新 OLED 显示
 */
static void display_update(void)
{
    float yaw   = GYRO_GetYaw();
    float wz    = GYRO_GetWz();
    bool  busy  = Corner_IsTurning();
    u8    count = Corner_GetCount();

    OLED_Clear();

    /* 第一行: Yaw + 状态 */
    OLED_ShowString(0, 0, (u8 *)"Y:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 0, yaw, 3, 2, FONT_SIZE);
    OLED_ShowChar(112, 0, 'd', FONT_SIZE);

    /* 第二行: Wz + 方向 */
    OLED_ShowString(0, 24, (u8 *)"W:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 24, wz, 4, 1, FONT_SIZE);

    /* 第三行: 状态 + 直角计数 */
    if (busy) {
        OLED_ShowString(0, 48, (u8 *)"TURNING...", 12);
    } else {
        OLED_ShowString(0, 48, (u8 *)"FOLLOW  #", 12);
        u8 digits[4];
        digits[0] = '0' + (count / 100) % 10;
        digits[1] = '0' + (count / 10)  % 10;
        digits[2] = '0' + (count)       % 10;
        digits[3] = '\0';
        OLED_ShowString(72, 48, digits, 12);
    }

    OLED_Refresh();
}

/* ================================================================ */
/*  main                                                             */
/* ================================================================ */
int main(void)
{
    SYSCFG_DL_init();

    /* ---- 外设模块初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 8,  (u8 *)"Square Run", 24);
    OLED_ShowString(0, 48, (u8 *)"Init...", 12);
    OLED_Refresh();
    delay_ms(800);

    Motor_Init();
    GYRO_Init();
    Corner_Init(CORNER_KP, CORNER_KI, CORNER_KD,
                CORNER_BASE_SPEED, CORNER_TRIM);

    display_update();

    /* ---- 主循环 ---- */
    while (1) {
        /*
         * 调用顺序至关重要:
         * 1. GYRO_Poll() — 高频轮询陀螺仪 UART, 保证 FIFO 不溢出
         * 2. Corner_Task() — 灰度循迹 + 直角检测 + 转弯
         * 两者都无阻塞, 快速返回, 确保陀螺仪数据持续更新
         */
        GYRO_Poll();
        Corner_Task();

        /* 定期刷新 OLED 显示 */
        static uint32_t loop_cnt = 0;
        if (++loop_cnt >= DISPLAY_LOOP_CNT) {
            loop_cnt = 0;
            display_update();
        }
    }
}
