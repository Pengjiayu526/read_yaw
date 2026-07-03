# 两轮差速小车工程文档 — MSPM0 原地直角转弯 + 陀螺仪闭环控制

## 工程概览

| 项目 | 详情 |
|------|------|
| MCU | TI MSPM0G3507 @ 32MHz |
| 电机驱动 | 2路 PWM (TIMG0), 单向前进, 差速 pivot turn |
| 姿态传感器 | M0G3507 MEMS 陀螺仪, UART 协议 |
| 显示 | SSD1306 OLED, 128×64, I2C 地址 0x3C |
| 转弯控制 | PD 闭环 + 状态机, 目标 ±2.5° 精度 |
| 开发环境 | TI Code Composer Studio Theia |

---

## 文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `motor.h` / `motor.c` | 电机驱动 | PWM 占空比控制, 双轮差速 |
| `gyro.h` / `gyro.c` | 陀螺仪 | UART 协议解析 Yaw/Wz |
| `turn.h` / `turn.c` | 转弯控制 | PD 状态机, 闭环直角转弯 |
| `oled.h` / `oled.c` | OLED 显示 | I2C SSD1306, 字符/图形 |
| `oledfont.h` | 字库 | ASCII 12/16/24 + 汉字 16/24/32/64 |
| `delay.h` / `delay.c` | 延时 | SysTick 精确定时 us/ms |
| `empty.c` | 主程序 | 自动测试: 间隔 1s 循环直角转弯 |
| `ti_msp_dl_config.h/c` | HAL 配置 | SysConfig 生成 (Debug 目录) |

---

## 硬件连接

### 电机 PWM
| 信号 | 引脚 | MSPM0 资源 |
|------|------|-----------|
| 右轮 PWM | PA12 | TIMG0 CCP0 |
| 左轮 PWM | PA13 | TIMG0 CCP1 |

- PWM 频率: 10 kHz (1.28 MHz 时钟 / 128 周期)
- 占空比映射: speed 0→0% (停止), speed 100→100% (全速)
- **仅支持前进, 不支持反转**

### 陀螺仪 UART
| 信号 | MSPM0 资源 |
|------|-----------|
| RX | UART_2 |

- 波特率: 115200 (默认)
- 协议帧: `0x5A + TYPE + DL + DH + SUM` (5 字节)
  - TYPE=0xAA: 角速度 Wz, 量程 ±2000°/s
  - TYPE=0xBB: 航向角 Yaw, 量程 ±180°
  - 校验: SUM = (0x5A + TYPE + DL + DH) & 0xFF

### OLED I2C
| 信号 | MSPM0 资源 |
|------|-----------|
| SCL/SDA | OLED_INST (硬件 I2C) |

- 地址: 0x3C
- 分辨率: 128×64, 页寻址模式

---

## 模块 API 速查

### 1. 电机驱动 (motor.h)

```c
void Motor_Init(void);
// 初始化 PWM, 启动定时器, 电机初始为停止状态
// 必须在 SYSCFG_DL_init() 之后调用

void Motor_SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed);
// 设置左右轮速度, 范围 0~100 (%)
// leftSpeed=0, rightSpeed=60 → 左轮停止, 右轮60% → 原地右转

void Motor_Stop(void);
// 两轮停止 (等价于 Motor_SetSpeed(0,0))
```

### 2. 陀螺仪 (gyro.h)

```c
void  GYRO_Init(void);          // 初始化状态机
void  GYRO_Poll(void);          // 轮询 UART FIFO, 解析数据 (需在主循环高频调用)
float GYRO_GetYaw(void);        // 获取航向角 (°), 范围 -180~+180
float GYRO_GetWz(void);         // 获取 Z 轴角速度 (°/s)
```

> **关键**: `GYRO_Poll()` 必须在 `GYRO_GetYaw()` / `GYRO_GetWz()` 之前调用, 否则读到的是旧数据。

### 3. 转弯控制 (turn.h)

```c
void Turn_Init(void);
// 初始化转弯状态机

void Turn_Start(int angle);
// 启动一次原地转弯
// angle > 0 → 顺时针 (右转), angle < 0 → 逆时针 (左转)
// 典型值: ±90, ±180
// 若正在转弯中 (Turn_IsBusy()==true) 则忽略本次调用

void Turn_Task(void);
// 转弯状态机任务, 需在主循环中高频调用
// 调用前必须先调 GYRO_Poll(), 函数内部无 delay 无阻塞

bool Turn_IsBusy(void);
// true=正在转弯, false=空闲
```

### 4. OLED 显示 (oled.h) — 常用函数

```c
void OLED_Init(void);                              // 初始化 OLED
void OLED_Clear(void);                             // 清屏
void OLED_Refresh(void);                           // 刷新显示 (将显存推到屏幕)
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size);   // 显示 ASCII 字符 (size=12/16/24)
void OLED_ShowString(u8 x, u8 y, u8 *str, u8 size); // 显示字符串
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size); // 显示无符号整数
void OLED_DrawPoint(u8 x, u8 y);                   // 画点
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);    // 画线
void OLED_DrawCircle(u8 x, u8 y, u8 r);            // 画圆
void OLED_ShowChinese(u8 x, u8 y, u8 idx, u8 size); // 显示汉字 (size=16/24/32/64)
```

画点函数修改的是显存 `OLED_GRAM[144][8]`, 需要调 `OLED_Refresh()` 才会显示到屏幕。

### 5. 延时 (delay.h)

```c
void delay_ms(unsigned long ms);  // 毫秒延时 (阻塞)
void delay_us(unsigned long us);  // 微秒延时 (阻塞)
```

> **注意**: delay 函数是阻塞的, 控制循环中**不要使用**, 只在初始化阶段使用。

---

## 主循环框架 (参考 empty.c)

```c
int main(void)
{
    SYSCFG_DL_init();     // TI SysConfig 生成的硬件初始化
    OLED_Init();          // OLED 初始化
    Motor_Init();         // 电机 PWM 初始化
    GYRO_Init();          // 陀螺仪解析初始化
    Turn_Init();          // 转弯状态机初始化

    while (1) {
        GYRO_Poll();      // ← 1. 必须先 Poll 更新陀螺仪数据
        Turn_Task();      // ← 2. 转弯状态机 (需 GYRO_Poll 之后)

        // 3. 你的应用逻辑 (如灰度循迹)
        // ...

        // 4. 定期刷新 OLED (用计数器而非 delay)
        static uint32_t disp_cnt = 0;
        if (++disp_cnt >= 50000) {
            disp_cnt = 0;
            // OLED 显示更新
        }
    }
}
```

**核心规则**:
1. `GYRO_Poll()` 在 `Turn_Task()` **之前**调用
2. 控制循环内**禁止**使用 `delay_ms()` (会阻塞陀螺仪数据更新)
3. 需要定时时用**迭代计数器**, 不要用 delay

---

## 转弯控制算法详解

### 状态机

```
Turn_Start() → IDLE → SPIN → STOP → IDLE
                ↑                 │
                └─────────────────┘
```

| 状态 | 说明 |
|------|------|
| IDLE | 空闲, 等待 Turn_Start() 指令 |
| SPIN | 旋转中, PD 闭环控制, 实时读取陀螺仪 |
| STOP | 刹车过渡态, 停电机后 1 次迭代回到 IDLE |

### PD 控制器公式

```
pd_out = Kp × error - Kd × Wz

其中:
  error = target_yaw - current_yaw  (已做 ±180° 归一化, 取最短路径)
  Wz    = 当前 Z 轴角速度 (°/s)
```

- **P 项** (`Kp × error`): 误差越大驱动力越强, 远离目标时加速
- **D 项** (`Kd × Wz`): 转速越快抑制力越强, 自动刹车/减速
- **`pd_out` 符号决定方向**: >0 → 右转 (左轮驱动), <0 → 左转 (右轮驱动)
- D 项 > P 项时 `pd_out` 变号 → 自动切对侧轮 → 主动刹车

### 差速驱动策略

由于电机**不支持反转**, 采用 **pivot turn** (支点转弯):
- 右转: 左轮前进, 右轮静止 → 车体绕右轮旋转
- 左转: 右轮前进, 左轮静止 → 车体绕左轮旋转

### 可调参数 (定义在 turn.c)

| 宏 | 默认值 | 说明 | 调大效果 |
|----|--------|------|---------|
| `TURN_KP` | 0.70 | P 项系数 | 加速更快, 易过冲 |
| `TURN_KD` | 0.18 | D 项系数 | 刹车更早, 后半程更慢 |
| `TURN_MIN_DRIVE` | 6 | 最小驱动力 (%) | 不易卡住, 可能微振 |
| `TURN_MAX_SPEED` | 60 | 最大驱动力 (%) | 转弯更快 |
| `TURN_ERR_THRESH` | 2.5 | 角度误差阈值 (°) | 放宽精度 |
| `TURN_WZ_THRESH` | 6.0 | 角速度阈值 (°/s) | 更容易停 |
| `TURN_TIMEOUT_TICKS` | 200000 | 总超时迭代数 | 更长时间才强制结束 |
| `TURN_SETTLE_TICKS` | 50000 | settle 超时迭代数 | 抗 Wz 偏置, 等更久 |

### 停止判定 (双重保险)

1. **正常停止**: `|error| < 2.5°` **且** `|Wz| < 6.0°/s` → 立即结束
2. **Settle 兜底**: 误差达标但 Wz 始终不达标 (陀螺仪偏置/噪声) → 连续 50000 次迭代后强制结束
3. **总超时**: SPIN 超过 200000 次迭代 → 强制结束

---

## 融合灰度循迹的注意事项

1. **主循环结构**: 保持 `GYRO_Poll()` → `Turn_Task()` → `你的循迹逻辑` → `OLED 刷新`
2. **不要用 delay**: 循迹判断也用迭代计数器做定时
3. **冲突处理**: 转弯时 (`Turn_IsBusy()==true`) 应暂停循迹, 转弯完成后再恢复
4. **电机控制冲突**: 转弯期间 `Turn_Task()` 会调用 `Motor_SetSpeed()`, 循迹逻辑**不要同时**调用 `Motor_SetSpeed()`
5. **典型融合模式**:
   ```c
   while (1) {
       GYRO_Poll();
       Turn_Task();

       if (!Turn_IsBusy()) {
           // 不转弯时跑循迹
           line_tracking_task();  // 你的循迹函数
       }
       // 转弯时 Turn_Task 自动接管电机, 循迹不干扰

       display_update();
   }
   ```
6. **转弯触发**: 循迹检测到路口/直角 → 调用 `Turn_Start(±90)` → 转弯期间暂停循迹 → `Turn_IsBusy()==false` 后恢复循迹

---

## 已知问题/注意事项

1. **OLED_ShowNum bug**: 数字 0 在所有位都显示为 '0', 如 "102" 可能显示异常。建议用 `OLED_ShowChar` 逐位显示或修复该函数。
2. **电机仅支持前进**: 不能反转, 刹车靠驱动对侧轮子的 D 项实现
3. **陀螺仪 Yaw 漂移**: 长时间运行 Yaw 可能漂移, 每次 `Turn_Start` 以当前 Yaw 为基准计算目标, 相对角度准确
4. **无操作系统**: 裸机程序, 所有任务在 while(1) 中轮询执行
5. **`delay_times` 变量**: delay.c 中定义但未使用, SysTick_Handler 已注释

---

## 编译说明

1. 用 TI CCS Theia 打开工程
2. `ti_msp_dl_config.c/h` 由 SysConfig 生成, 配置了 TIMG0、UART_2、I2C 等外设
3. 所有 `.c` 文件需加入工程编译
4. 确保 `MOTOR_PWM_PERIOD=128` 与 SysConfig 中 TIMG0 的 period 一致
