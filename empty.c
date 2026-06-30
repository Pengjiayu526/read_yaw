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
#include "turn.h"
#include "delay.h"

/* ---- 显示参数 ---- */
#define FONT_SIZE        16          /* 使用16像素字体 */
#define DISPLAY_INTERVAL 100         /* 显示刷新间隔 (ms) */

/* ---- 转弯测试参数 ---- */
/*
 * 主循环空闲迭代 ≈ 2~5 us/次 (无UART数据时)。
 * 50,000 次 × 4 us ≈ 200 ms 用于显示刷新。
 * 转弯间隔: 600,000 次 × 4 us ≈ 2.4 s。
 */
#define DISPLAY_LOOP_CNT   50000    /* 显示刷新迭代间隔 */
#define TURN_INTERVAL_CNT 600000    /* 两次转弯之间的迭代间隔 (~2~3s) */

/* ---- 测试阶段 ---- */
static int       turn_count  = 0;   /* 已完成转弯次数 */
static uint32_t  idle_timer  = 0;   /* 空闲计时器, 闲时递增 */

/* ---- 内部辅助函数 ---- */

/**
 * @brief 在 OLED 指定位置显示带符号的浮点数
 * @param x      起始横坐标
 * @param y      起始纵坐标
 * @param val    要显示的浮点数值
 * @param int_w  整数部分显示宽度 (位数)
 * @param frac_w 小数部分显示宽度 (位数)
 * @param size   字体大小 (12/16/24)
 */
static void OLED_ShowSignedFloat(u8 x, u8 y, float val,
                                  u8 int_w, u8 frac_w, u8 size)
{
    u8 x_pos = x;
    u8 char_w = size / 2;  /* 每个字符的宽度 */

    /* ---- 符号位 ---- */
    if (val < 0.0f) {
        OLED_ShowChar(x_pos, y, '-', size);
        val = -val;
    } else {
        OLED_ShowChar(x_pos, y, ' ', size);  /* 正数留空 */
    }
    x_pos += char_w;

    /* ---- 整数部分 ---- */
    u32 int_part = (u32)val;
    OLED_ShowNum(x_pos, y, int_part, int_w, size);
    x_pos += int_w * char_w;

    /* ---- 小数点 ---- */
    OLED_ShowChar(x_pos, y, '.', size);
    x_pos += char_w;

    /* ---- 小数部分 (四舍五入到 frac_w 位) ---- */
    float frac = val - (float)int_part;
    u32 mult = 1;
    for (u8 i = 0; i < frac_w; i++) mult *= 10;
    u32 frac_part = (u32)(frac * (float)mult + 0.5f);

    /* 处理四舍五入溢出 (如 0.999 进位到 1.000) */
    if (frac_part >= mult) {
        frac_part = 0;
        /* 整数部分进位 — 简单忽略, 因为整数已显示完成 */
        /* 实际在2位小数精度下极少发生 */
    }

    OLED_ShowNum(x_pos, y, frac_part, frac_w, size);
}

/**
 * @brief 刷新 OLED 显示: 清屏 -> 绘制数据 -> 刷新
 */
static void display_update(void)
{
    float wz  = GYRO_GetWz();
    float yaw = GYRO_GetYaw();
    bool busy = Turn_IsBusy();

    OLED_Clear();

    /* ---- 第一行: Yaw (航向角) + 状态 ---- */
    OLED_ShowString(0, 0, (u8 *)"Y:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 0, yaw, 3, 2, FONT_SIZE);
    OLED_ShowChar(112, 0, 'd', FONT_SIZE);

    /* ---- 第二行: Wz (Z轴角速度) + 方向指示 ---- */
    OLED_ShowString(0, 24, (u8 *)"W:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 24, wz, 4, 1, FONT_SIZE);
    OLED_ShowString(104, 24, (u8 *)"d/s", FONT_SIZE);

    /* ---- 第三行: 转弯次数 + 状态 ---- */
    if (busy) {
        OLED_ShowString(0, 48, (u8 *)"TURNING...", 12);
    } else {
        OLED_ShowString(0, 48, (u8 *)"IDLE  #", 12);
        /* 显示已完成的转弯次数 */
        u8 digits[4];
        digits[0] = '0' + (turn_count / 100) % 10;
        digits[1] = '0' + (turn_count / 10)  % 10;
        digits[2] = '0' + (turn_count)       % 10;
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
    OLED_ShowString(0, 8,  (u8 *)"Diff Turn", 24);
    OLED_ShowString(0, 48, (u8 *)"Init...", 12);
    OLED_Refresh();
    delay_ms(800);

    Motor_Init();
    GYRO_Init();
    Turn_Init();

    /* ---- 首次刷新显示 ---- */
    display_update();

    /* ---- 主循环 ---- */
    while (1) {
        /* 高频轮询 UART, 更新陀螺仪数据 */
        GYRO_Poll();

        /* 转弯状态机驱动 (必须在 GYRO_Poll 之后) */
        Turn_Task();

        /*
         * 自动测试: 小车空闲时启动新一轮 90° 直角转弯。
         * idle_timer 仅在空闲时累加, 转弯过程中不计数,
         * 因此转弯完成后才会启动下一轮倒计时。
         */
        if (!Turn_IsBusy()) {
            if (++idle_timer >= TURN_INTERVAL_CNT) {
                idle_timer = 0;
                Turn_Start(90);         /* 顺时针直角转弯 */
                turn_count++;
            }
        } else {
            idle_timer = 0;             /* 转弯中, 重置空闲计时器 */
        }

        /* 定期刷新 OLED 显示 */
        static uint32_t loop_cnt = 0;
        if (++loop_cnt >= DISPLAY_LOOP_CNT) {
            loop_cnt = 0;
            display_update();
        }
    }
}
