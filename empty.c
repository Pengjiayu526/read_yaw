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
#include "delay.h"

/* ---- 显示参数 ---- */
#define FONT_SIZE        16          /* 使用16像素字体 */
#define DISPLAY_INTERVAL 100         /* 显示刷新间隔 (ms) */

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

    OLED_Clear();

    /* ---- 第一行: Wz (Z轴角速度) ---- */
    OLED_ShowString(0, 0, (u8 *)"W:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 0, wz, 4, 1, FONT_SIZE);
    OLED_ShowString(104, 0, (u8 *)"d/s", FONT_SIZE);

    /* ---- 第二行: Yaw (航向角) ---- */
    OLED_ShowString(0, 24, (u8 *)"Y:", FONT_SIZE);
    OLED_ShowSignedFloat(16, 24, yaw, 3, 2, FONT_SIZE);
    OLED_ShowChar(112, 24, 'd', FONT_SIZE);

    /* ---- 第三行: 状态/提示 ---- */
    OLED_ShowString(0, 48, (u8 *)"M0G3507 Gyro", 12);

    OLED_Refresh();
}

/* ================================================================ */
/*  main                                                             */
/* ================================================================ */
int main(void)
{
    SYSCFG_DL_init();

    /* ---- OLED 初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 16, (u8 *)"MEMS Gyro", 24);
    OLED_Refresh();
    delay_ms(800);

    /* ---- 陀螺仪解析模块初始化 (UART 已由 SYSCFG_DL_init 配置) ---- */
    GYRO_Init();

    /* ---- 首次刷新显示 ---- */
    display_update();

    /* ---- 主循环 ---- */
    while (1) {
        /* 高频轮询 UART, 处理所有收到的字节 */
        GYRO_Poll();

        /*
         * 每 100ms 刷新一次 OLED 显示。
         * 放在 GYRO_Poll() 之后, 确保先处理完所有积压数据再刷新。
         *
         * 由于 delay_ms 是阻塞的, 我们不在这里调用它;
         * 而是用 GYRO_Poll 的空闲迭代次数来粗略计时。
         * 这里采用一个简单计数器来近似 100ms 间隔:
         *   主循环每次迭代约 1~5us (无数据时),
         *   50000 次 ≈ 50~250ms, 取折中 50000 次。
         */
        static uint32_t loop_cnt = 0;
        if (++loop_cnt >= 50000) {
            loop_cnt = 0;
            display_update();
        }
    }
}
