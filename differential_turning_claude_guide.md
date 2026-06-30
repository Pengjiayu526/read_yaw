# Claude 开发指导：两轮差速小车90°原地转弯

## 目标
将现有电机PWM驱动与UART姿态角读取融合，实现稳定的90°原地转弯。

## 已有模块
- motor.c/h：Motor_Init、Motor_SetSpeed(left,right)、Motor_Stop()
- gyro.c/h：GYRO_Poll()、GYRO_GetYaw()、GYRO_GetWz()
- delay：仅初始化使用，控制过程中避免delay阻塞。

## 核心思想
采用状态机+闭环控制，不使用延时估计角度。

数据流：
GYRO_Poll() -> GYRO_GetYaw() -> 角度误差 -> 控制器 -> Motor_SetSpeed()

## 推荐新增文件
turn.h
turn.c

提供接口：
```
void Turn_Init(void);
void Turn_Start(int angle);
void Turn_Task(void);
bool Turn_IsBusy(void);
```

## 状态机
IDLE
↓
记录起始Yaw
↓
SPIN(持续读取Yaw并控制)
↓
误差<2°且角速度接近0
↓
STOP
↓
IDLE

## 角度处理
必须处理±180°跳变：

```
error = target-current;
while(error>180) error-=360;
while(error<-180) error+=360;
```

## 控制策略
第一版建议P控制：

output=Kp*error

限制最小速度避免堵转，限制最大速度保护电机。

左转：
left=0;
right=output;

若支持反转：
left=-output;
right=output;

## main循环

```
while(1){
 GYRO_Poll();
 Turn_Task();
}
```

禁止Turn_Task内部使用delay。

## 调参建议
停止条件：
- abs(error)<2°
- abs(Wz)<5°/s

建议速度40~60%。

## 第二阶段
第一版成功后再加入：
- PI/PID
- 转弯超时
- 自动校准
- 前进+转弯组合
