# STC32G 电机控制系统 — 源码详细讲解

---

# 第一部分：上位机（motor_master.py）

## 一、文件概览

本文件是 STC32G 电机控制板的 **Modbus RTU 上位机**，使用 Python + Tkinter 实现，通过串口与下位机通信，实现电机状态监测、参数设定和 PID 在线调参。

整体架构分为四层：

```
┌─────────────────────────────────┐
│         GUI 界面层 (App)         │  Tkinter 窗口、按钮、输入框
├─────────────────────────────────┤
│       业务逻辑层 (App 方法)       │  轮询、写入、PID读写
├─────────────────────────────────┤
│      串口通信层 (ModbusSerial)    │  发送/接收帧、线程锁
├─────────────────────────────────┤
│     协议层 (CRC/帧构造函数)       │  CRC16、帧组装、帧校验
└─────────────────────────────────┘
```

---

## 二、协议层（第 16~104 行）

### 2.1 Modbus 协议常量（第 16~53 行）

```python
SLAVE_ADDR      = 0x01          # 从站地址，与下位机 rs485.h 中一致
FC_READ_COILS   = 0x01          # 功能码：读线圈
FC_READ_REGS    = 0x03          # 功能码：读保持寄存器
FC_WRITE_COIL   = 0x05          # 功能码：写单个线圈
FC_WRITE_REG    = 0x06          # 功能码：写单个寄存器
FC_WRITE_MULTI  = 0x10          # 功能码：写多个寄存器
```

**寄存器地址映射**（与下位机 `modbus_regs[]` 数组下标一一对应）：

| 地址 | 名称 | 读写 | 说明 |
|------|------|------|------|
| 0x0000 | Motor_Angle | 只读 | 当前角度（0.1°单位，带符号） |
| 0x0001 | Motor_Speed | 只读 | 当前转速（rpm，带符号） |
| 0x0002 | Encoder_H | 只读 | 编码器累计高16位 |
| 0x0003 | Encoder_L | 只读 | 编码器累计低16位 |
| 0x0004 | Set_Speed | 读写 | 目标转速 |
| 0x0005 | Set_Angle | 读写 | 目标角度（0.1°单位） |
| 0x0006 | Control_Mode | 读写 | 0=速度 1=角度 2=手动 |
| 0x0007 | Status_Flags | 只读 | bit0=到位 bit1=超载 bit2=方向 |
| 0x0008 | PWM_Duty_Raw | 读写 | 手动模式PWM值 |
| 0x0009 | ms_Tick | 只读 | 系统时基 |
| 0x000A~0x000F | PID参数 | 读写 | 速度/角度环 Kp/Ki/Kd（×100整数） |

### 2.2 CRC-16 校验（第 56~72 行）

Modbus RTU 每帧末尾都有 2 字节 CRC 校验，用于检测传输错误。

```python
def crc16(data: bytes) -> int:
    crc = 0xFFFF                    # 初始值 0xFFFF
    for b in data:
        crc ^= b                    # 逐字节异或
        for _ in range(8):          # 每个bit执行一次
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc
```

**算法原理**：Modbus 使用多项式 `0xA001`（即 `0x8005` 的位反转），对每个字节的每个 bit 做移位异或运算。这是标准的查表法的等价展开形式。

`build_frame` 将 CRC 追加到数据尾部（低字节在前），`check_crc` 验证接收帧的 CRC 是否正确。

### 2.3 帧构造函数（第 74~104 行）

四个函数分别构造四种 Modbus 请求帧：

**`req_read_regs(addr, count)`** — 读保持寄存器（FC=0x03）
```
帧格式：[从站地址][03][起始地址高][起始地址低][数量高][数量低][CRC-L][CRC-H]
```

**`req_write_reg(addr, value)`** — 写单个寄存器（FC=0x06）

**`req_write_coil(addr, on)`** — 写单个线圈（FC=0x05），ON=0xFF00，OFF=0x0000

**`req_write_multi(addr, values)`** — 写多个寄存器（FC=0x10），用于批量写入

**`to_int16(val)`** — 无符号到有符号转换：Modbus 寄存器是 16 位无符号，负数需要减 65536 还原。

---

## 三、串口通信层（第 106~177 行）

### ModbusSerial 类

`send_recv(frame, expect_len)` 是核心方法，流程：
1. `with self.lock:` — 加锁防止轮询和手动写入并发冲突
2. `reset_input_buffer()` — 清空残留数据
3. `write(frame)` — 发送请求
4. `sleep(0.01)` — 等 10ms 帧间隔
5. `read(expect_len)` — 读取应答（150ms 超时）
6. 三层校验：长度≥4、CRC 正确、非异常响应

高层方法 `read_regs` / `write_reg` / `write_coil` / `write_multi` 封装了帧构造和应答解析。

---

## 四、GUI 界面层（第 179~435 行）

窗口三栏布局：左栏状态监测（角度/速度/编码器/状态灯/轮询开关），中栏电机控制（模式选择/目标设定/线圈开关），右栏 PID 调参（速度环/角度环各 Kp/Ki/Kd）。

底部通信日志支持四色标签（tx 青色、rx 绿色、err 红色、inf 紫色）。

---

## 五、业务逻辑层（第 437~672 行）

轮询用 `after(200, callback)` 实现非阻塞定时；设定值写入用 FC=0x10 批量写 3 个寄存器；PID 参数输入实际浮点值 ×100 后写入下位机。

---

## 六、数据流总结

```
读取：上位机 TX [01 03 00 00 00 10 CRC] → 下位机应答 [01 03 20 数据32B CRC] → 解析更新GUI
写入：上位机 TX [01 10 00 04 00 03 06 数据6B CRC] → 下位机写入regs[] → 应答确认
```

---
---

# 第二部分：下位机源码讲解

## 一、系统总览

下位机运行在 STC32G 单片机上，主频 33.1776MHz，负责电机的实时闭环控制。整个系统由 7 个源文件组成：

```
┌──────────────────────────────────────────────────────┐
│                    main.c（主控）                      │
│  系统初始化、主循环、中断服务、PID控制、OLED显示、寄存器同步  │
├──────────┬──────────┬──────────┬──────────┬───────────┤
│ timer.c  │  uart.c  │ rs485.c  │  key.c   │  pid.c    │
│ 定时器    │  串口    │ Modbus   │  按键    │  PID算法   │
│ PWM输出   │  收发    │ 从站协议  │  蜂鸣器  │           │
│ 编码器    │  Printf  │ CRC校验  │  状态机  │           │
└──────────┴──────────┴──────────┴──────────┴───────────┘
```

**执行流程**：

```
上电 → Sys_init/CLK_Init → 各外设初始化 → 进入 while(1) 主循环
                                                │
        ┌───────────────────────────────────────┤
        │                                       │
  主循环（非实时）                         Timer0中断（1ms，实时）
  ├─ Modbus_SyncRegs()                    ├─ ms_tick++
  ├─ Modbus_Poll()                        ├─ Modbus_TimerTick()
  ├─ Key_HandleEvent()                    ├─ Key_Scan()
  ├─ OLED_Update()  (每20ms)              ├─ Buzzer_Update()
  └─ Printf调试    (每50ms)              └─ 每10ms:
                                              ├─ Encoder_Read()
                                              ├─ 速度/角度计算
                                              └─ Motor_ControlUpdate()
```

关键设计：**实时控制放在中断里（10ms 周期），非实时通信和显示放在主循环里**。中断只设标志位，主循环检测标志后执行耗时操作（如 OLED 刷屏、Printf）。

---

## 二、main.h — 系统级宏定义

```c
#define FOSC    33177600UL          // 系统主频 33.1776MHz
```

这个频率不是随意选的——它是 115200 波特率的整数倍（33177600 / 4 / 115200 = 72），所以 UART 波特率精确无误差。

```c
#define PWM_ARR   (FOSC / PWM_FREQ)     // = 3317
#define PWM_MID   (PWM_ARR / 2)         // = 1658
```

**PWM_ARR** 是 PWM 计数器的自动重装载值，决定 PWM 频率：33177600 / 3317 = 10kHz。

**PWM_MID** 是占空比的中点（50%）。H 桥驱动中，CCR=PWM_MID 时两臂电压相等，电机净电压为零（停车）。CCR 偏离中点越多，电机转速越快；偏离方向决定正反转。

```c
#define T0_RELOAD   (65536 - FOSC / 1000)   // = 32359
```

Timer0 重载值，产生 1ms 中断：33177600 个时钟 / 1000 = 33177.6 ≈ 33177 个计数，从 (65536 - 33177) = 32359 开始计数到 65535 溢出，刚好 1ms。

---

## 三、timer.c — 定时器、PWM、编码器

### 3.1 Timer0_Init — 1ms 系统心跳

```c
TMOD &= 0xF0;       // 模式0：16位自动重载
AUXR |= 0x80;       // 1T模式（不分频，直接用系统时钟计数）
TH0 = T0_RELOAD >> 8;
TL0 = T0_RELOAD & 0xFF;
TR0 = 1;             // 启动
```

STC32G 的"1T模式"意思是每个系统时钟都计数一次（传统 8051 是每 12 个时钟计一次）。所以 33.1776MHz 的 1T 模式下，定时精度远高于传统 12T 模式。

### 3.2 PWM_Init — 互补 PWM 输出（电机驱动）

PWMA 模块配置为边沿对齐、向上计数模式，通道 1 输出互补 PWM（PWM1P + PWM1N）：

```c
PWMA_CR1 = 0x80;           // ARPE=1（预装载使能），向上计数
PWMA_CCMR1 = 0x68;         // PWM模式1 + CCR预装载
PWMA_CCER1 = 0x05;         // 正向+互补输出均使能
PWMA_BKR = 0xC0;           // MOE=1（主输出使能，不设此位PWM无输出！）
PWMA_DTR = 60;              // 死区时间 ≈ 1.8us
```

**死区时间**是互补 PWM 必须有的安全间隔。如果 PWM1P 和 PWM1N 同时导通（即使只有几十纳秒），H 桥上下管直通会短路烧毁。死区保证一管关断后延迟 1.8us 另一管才开启。

**`PWM_SetDuty(duty)`** 写入 CCR1 寄存器改变占空比：

```c
void PWM_SetDuty(unsigned int duty)
{
    P_SW2 |= 0x80;          // 使能扩展SFR访问（CCR1属于扩展区）
    PWMA_CCR1H = duty >> 8;
    PWMA_CCR1L = duty & 0xFF;
    P_SW2 &= ~0x80;         // 关闭扩展SFR访问
}
```

`P_SW2 |= 0x80` 是 STC32G 的特殊操作：PWMA 的寄存器在扩展 SFR 区域，必须先打开"EAXFR"位才能访问，用完再关上。

### 3.3 PWMB_Encoder_Init — 编码器接口

利用 PWMB 定时器的编码器模式，硬件自动计数：

```c
PWMB_CCMR1 = 0xF1;    // TI1输入 + 最大滤波（7.7us）
PWMB_CCMR2 = 0xF1;    // TI2输入 + 最大滤波
PWMB_SMCR  = 0x03;     // 编码器模式3：A/B两相都计数（四倍频）
PWMB_CR1   = 0x81;     // 启动计数器
```

**编码器工作原理**：电机轴上的编码器盘输出 A/B 两相正交方波。正转时 A 相超前 B 相 90°，反转时 B 相超前 A 相。硬件编码器模式自动判断方向，A/B 每个边沿都计数（4 倍频），11 线编码器每转产生 44 个脉冲。

**滤波 `0xF1`**：编码器信号线和 PWM 驱动线在同一块 PCB 上，10kHz 的 PWM 开关噪声会耦合到编码器输入引脚上产生毛刺。`0xF1` 设置最大硬件滤波（32个采样时钟 ≈ 7.7us），把短于此的毛刺过滤掉。

**`Encoder_Read()`** 读取当前计数值并转为有符号：

```c
raw = (PWMB_CNTRH << 8) | PWMB_CNTRL;
if (raw & 0x8000)
    cnt = -(int)(0x10000 - raw);   // 补码转换：0xFFFF → -1
else
    cnt = (int)raw;
```

---

## 四、uart.c — 双串口通信

### 4.1 UART1 — 调试串口（TTL 电平）

```c
P_SW1 |= 0x40;     // 串口1切换到 P3.6(RXD)/P3.7(TXD)
SCON = 0x50;        // 模式1（8位UART），允许接收
AUXR |= 0x40;       // 定时器1 使用 1T 模式
```

UART1 用于 `Printf` 调试输出，直连 USB 转 TTL 模块到电脑串口助手查看。发送是阻塞方式（`while(!TI)` 等待发完）。

### 4.2 UART2 — Modbus 通信串口（RS485）

```c
S2CON = 0x50;       // 8位模式，允许接收
AUXR |= 0x14;       // 定时器2 1T模式 + 启动
S2_S = 1;           // 串口2切换到 P4.6(RXD2)/P4.7(TXD2)
```

UART2 的发送采用**中断驱动环形缓冲区**：

```c
void UART2_SendByte(unsigned char dat)
{
    // 把数据放入环形缓冲区
    uart2_tx_buf[uart2_tx_head] = dat;
    uart2_tx_head = (uart2_tx_head + 1) % UART2_BUF_SIZE;

    // 如果发送空闲，立即启动第一个字节
    if (!uart2_tx_busy) {
        uart2_tx_busy = 1;
        S2BUF = uart2_tx_buf[uart2_tx_tail];
        uart2_tx_tail = (uart2_tx_tail + 1) % UART2_BUF_SIZE;
    }
    // 后续字节由 UART2_ISR 中断自动发送
}
```

**为什么不用阻塞发送？** Modbus 应答可能有几十字节，如果每字节都 `while(!TI)` 等待，会占用大量 CPU 时间，影响主循环和中断的响应。环形缓冲区+中断发送是非阻塞的——`SendByte` 只是把数据放入缓冲区就返回，硬件发完一个字节后中断自动取下一个继续发。

### 4.3 Printf — 可变参数调试输出

```c
void Printf(const char *fmt, ...)
{
    char buf[100];
    va_list args;
    va_start(args, fmt);
    vsprintf(buf, fmt, args);    // 格式化到缓冲区
    va_end(args);
    UART1_SendStr(buf);          // 通过UART1发送
}
```

这是对 C 标准库 `vsprintf` 的封装，让单片机也能像 PC 一样用 `Printf("Speed=%d\n", speed)` 打印调试信息。

---

## 五、rs485.c — Modbus RTU 从机实现

### 5.1 数据存储

```c
unsigned int  modbus_regs[16];     // 16个保持寄存器（0x0000~0x000F）
unsigned char modbus_coils = 0x00; // 8个线圈（bit0~bit7）
```

这两个数组/变量就是 Modbus 协议的"数据模型"。上位机通过功能码读写它们，主程序通过 `Modbus_SyncRegs()` 和它们交换数据。它们是上位机和下位机之间的**共享内存桥梁**。

### 5.2 帧接收 — 中断收字节 + 超时判帧

Modbus RTU 没有帧头帧尾标记，靠**3.5 个字符时间的静默**来分隔帧。实现分三层协作：

**UART2 中断（每收到1字节触发）：**
```c
void UART2_ISR(void) interrupt 8 {
    if (S2CON & 0x01) {          // 收到一个字节
        S2CON &= ~0x01;          // 清接收标志
        Modbus_RxByte(S2BUF);   // 送给Modbus状态机
    }
}
```

**Modbus_RxByte（在中断中调用）：**
```c
void Modbus_RxByte(unsigned char dat) {
    timeout_cnt = 0;      // 每收到一字节，重置超时计数器
    rx_active = 1;        // 标记"正在接收帧"
    rx_buf[rx_len++] = dat;
}
```

**Modbus_TimerTick（Timer0 中每1ms调用）：**
```c
void Modbus_TimerTick(void) {
    if (rx_active) {
        timeout_cnt++;
        if (timeout_cnt >= 2) {    // 2ms无新字节 → 帧结束
            rx_done = 1;           // 通知主循环处理
            rx_active = 0;
        }
    }
}
```

**三者协作流程**：收到第一个字节 → `rx_active=1`，开始计时 → 后续字节不断重置计时器 → 最后一个字节后2ms无新数据 → `rx_done=1` → 主循环 `Modbus_Poll()` 处理。

### 5.3 帧处理 — ProcessFrame

```c
static void ProcessFrame(void)
{
    // 1. 长度检查（至少4字节）
    if (rx_len < 4) return;

    // 2. 地址过滤（只响应自己的地址 0x01）
    if (rx_buf[0] != MODBUS_SLAVE_ADDR) return;

    // 3. CRC校验
    crc_recv = (rx_buf[rx_len-1] << 8) | rx_buf[rx_len-2];
    crc_calc = CRC16(rx_buf, rx_len - 2);
    if (crc_recv != crc_calc) return;    // CRC错误，静默丢弃

    // 4. 按功能码分发处理
    switch (rx_buf[1]) {
        case 0x01: /* 读线圈 */        ...
        case 0x03: /* 读保持寄存器 */   ...
        case 0x05: /* 写单个线圈 */     ...
        case 0x06: /* 写单个寄存器 */   ...
        case 0x10: /* 写多个寄存器 */   ...
        default:   SendException(...);  // 不支持的功能码
    }
}
```

**FC=0x03 读保持寄存器**的应答构造：
```c
for (i = 0; i < quantity; i++) {
    tx_buf[3 + i*2]     = modbus_regs[start_addr + i] >> 8;    // 高字节
    tx_buf[3 + i*2 + 1] = modbus_regs[start_addr + i] & 0xFF;  // 低字节
}
SendResponse(3 + quantity * 2);
```

**FC=0x10 写多个寄存器**的数据解析：
```c
for (i = 0; i < quantity; i++) {
    modbus_regs[start_addr + i] =
        (rx_buf[7 + i*2] << 8) | rx_buf[7 + i*2 + 1];  // 大端序合并
}
```

**异常响应**：当地址越界或参数非法时，回复功能码加 0x80 的异常帧：
```c
static void SendException(unsigned char func, unsigned char ex_code) {
    tx_buf[1] = func | 0x80;    // 0x03 → 0x83 表示读寄存器异常
    tx_buf[2] = ex_code;        // 0x02 = 非法地址
}
```

---

## 六、pid.c — 位置式 PID 控制器

### 6.1 核心公式

```
u[k] = (Kp × e[k] + Ki × Σe + Kd × (e[k] - e[k-1])) / 100
```

Kp/Ki/Kd 放大 100 倍存储（整数运算），最后除 100 还原。这样 `Kp=2.50` 存为 `250`，完全避免浮点运算（8051 架构没有硬件浮点单元，软浮点极慢）。

### 6.2 PID_Calc 逐行解析

```c
long PID_Calc(PID_t *pid, int setpoint, int feedback)
{
    int err = setpoint - feedback;          // 误差 = 目标 - 实际

    // 死区：误差太小时不输出，防止电机在目标附近来回抖动
    if (err <= pid->dead_band && err >= -pid->dead_band)
        err = 0;

    pid->integral += (long)err;             // 积分累加

    // 抗积分饱和：限制积分器大小，防止长时间误差累积导致巨大超调
    if (pid->Ki > 0) {
        long ilimit = pid->out_max * 100L / (long)pid->Ki;
        if (pid->integral >  ilimit) pid->integral =  ilimit;
        if (pid->integral < -ilimit) pid->integral = -ilimit;
    }

    int derivative = err - pid->e_prev;     // 微分 = 本次误差 - 上次误差
    pid->e_prev = err;

    // 位置式PID输出
    long output = ((long)pid->Kp * err
                 + (long)pid->Ki * pid->integral
                 + (long)pid->Kd * derivative) / 100L;

    // 输出限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
```

**抗积分饱和**的含义：假设 Ki=100（实际 1.00），out_max=1658，则 ilimit = 1658 × 100 / 100 = 1658。这保证积分项 `Ki × integral / 100` 不超过输出限幅，防止积分器"充"得太满，导致目标改变后还要很长时间才能"放"完。

---

## 七、key.c — 按键状态机 + 蜂鸣器

### 7.1 按键扫描状态机

`Key_ScanSingle` 每 1ms 在 Timer0 中断里调用一次，实现去抖、单击、双击、长按识别：

```
按下（GPIO=0）                          释放（GPIO=1）
    │                                      │
    ├─ debounce 20ms → state=1             ├─ debounce 20ms → state=0
    │                                      │
    ├─ press_time < 1000ms:                ├─ 短按释放:
    │   （等待释放）                        │   click_count++
    │                                      │   第1次 → event=SINGLE_CLICK
    ├─ press_time == 1000ms:               │           启动300ms双击定时器
    │   event=LONG_PRESS                   │   第2次 → event=DOUBLE_CLICK
    │                                      │
    ├─ press_time > 1000ms 每100ms:        ├─ 长按释放:
    │   event=PRESSING                     │   event=RELEASE
```

**去抖原理**：机械按键按下/释放时触点会弹跳几毫秒，产生多次 0/1 切换。去抖计数器要求连续 20ms 都是同一状态才确认状态改变。

### 7.2 按键功能

- **KEY1 单击**：在"目标速度"和"目标角度"之间切换调节对象（OLED 反白显示当前选中项）
- **KEY2 单击/双击/长按**：增大当前调节目标（步长分别为 小/大/中）
- **KEY3 单击/双击/长按**：减小当前调节目标

`Key_AdjustTarget(step)` 直接修改 `modbus_regs[]` 中的对应寄存器，`Modbus_SyncRegs()` 会在下一个主循环周期读取到新值。

### 7.3 蜂鸣器状态机

非阻塞音效播放，由音效段表驱动：

```c
// 开机音效：100ms响 → 80ms停 → 150ms响 → 80ms停 → 200ms响
{100, 1}, {80, 0}, {150, 1}, {80, 0}, {200, 1}, {0, 0}
```

`Buzzer_Update()` 每 1ms 在中断中检查当前段是否到时间，到了就切换到下一段。`duration_ms=0` 是结束标记。`Buzzer_PlayTone()` 只是设置起始状态就返回，完全非阻塞。

---

## 八、main.c — 系统核心

### 8.1 编码器参数

```c
#define ENC_LINES       11      // 编码器物理线数
#define ENC_MULT        4       // AB相四倍频
#define GEAR_RATIO      60      // 减速比
#define ENC_PPR_OUTPUT  2640    // 输出轴每转脉冲数 = 11×4×60
```

**速度公式推导**：采样周期 10ms，一次采样间的脉冲增量为 `enc_delta`。
```
speed(rpm) = enc_delta × (1000ms/10ms) × 60s / ENC_PPR_OUTPUT
           = enc_delta × 6000 / 2640
```
`6000 = 100(每秒采样次数) × 60(秒→分钟)`。

**角度公式**：累计脉冲 × 360° / 每转脉冲 = enc_total × 3600 / 2640（0.1°单位）。加上 `ANGLE_OFFSET=600`（60°偏移），使工作区间避开 0°/360° 跳变点。

### 8.2 Modbus_SyncRegs — 寄存器双向同步

这个函数是上位机通信和实时控制之间的桥梁，每个主循环周期执行一次：

```c
// 方向1：下位机状态 → 寄存器（上位机可读）
modbus_regs[0] = g_motor_angle;    // 当前角度
modbus_regs[1] = g_motor_speed;    // 当前转速
modbus_regs[7] = g_status_flags;   // 状态标志

// 方向2：寄存器（上位机可写）→ 控制变量
g_set_speed = (int)modbus_regs[4];  // 目标速度
g_set_angle = (int)modbus_regs[5];  // 目标角度
g_ctrl_mode = modbus_regs[6];       // 控制模式

// 方向2：PID参数寄存器 → PID结构体
g_pid_speed.Kp = (int)modbus_regs[0x0A];
```

### 8.3 Motor_ControlUpdate — 电机控制核心

每 10ms 在 Timer0 中断中调用，是整个系统的"心脏"：

**模式 0 — 速度单环**：
```
目标速度 ──→ [速度PID] ──→ PWM占空比 ──→ 电机
                ↑
           实际速度（编码器）
```
PID 输出 `pid_out` 通过 `duty = PWM_MID - pid_out` 映射到 CCR：正输出→CCR减小→正转，负输出→CCR增大→反转。

**模式 1 — 角度串级（位置环 + 速度环）**：
```
目标角度 ──→ [位置PID] ──→ 目标速度 ──→ [速度PID] ──→ PWM ──→ 电机
                ↑                          ↑
           实际角度                     实际速度
```
两级串联：位置环根据角度误差输出一个目标速度（限幅±100rpm），速度环再把这个目标速度转为 PWM 占空比。这样速度环负责"怎么转"，位置环负责"转到哪"。

**模式 2 — 手动**：直接把上位机写入的 `PWM_Duty_Raw` 值送给 PWM，绕过 PID。用于调试和校准。

### 8.4 Timer0_ISR — 1ms 定时中断

```c
void Timer0_ISR(void) interrupt 1
{
    ms_tick++;                      // 系统时基（全局时间戳）
    Modbus_TimerTick();             // Modbus帧超时检测
    Key_Scan();                     // 按键扫描（需要精确1ms节拍）
    Buzzer_Update();                // 蜂鸣器状态机推进

    if (ms_tick % 10 == 0)          // 每10ms：
    {
        // 1. 读编码器
        enc_now = Encoder_Read();
        enc_delta = enc_now - g_enc_last;

        // 2. 累计脉冲
        g_enc_total += enc_delta;

        // 3. 速度计算（滑动平均，每4次采样输出一次）
        g_speed_acc += enc_delta;
        if (++g_speed_idx >= 4) {
            g_motor_speed = g_speed_acc * 6000 / 2640 / 4;
            g_speed_acc = 0; g_speed_idx = 0;
        }

        // 4. 角度计算
        g_motor_angle = enc_total * 3600 / 2640 + ANGLE_OFFSET;

        // 5. PID控制输出
        Motor_ControlUpdate();
    }

    if (ms_tick % 20 == 0) disp_flag = 1;    // OLED刷新标志
    if (ms_tick % 50 == 0) debug_flag = 1;    // 串口调试标志
}
```

**为什么 PID 放在中断里而不是主循环？** PID 控制要求严格的固定周期（10ms）。如果放在主循环里，OLED 刷屏（约 5ms）、Modbus 通信等操作会导致控制周期不稳定，PID 的微分项和积分项都会出错。放在中断里保证了精确的 10ms 调用间隔。

### 8.5 主循环

```c
while (1)
{
    Modbus_SyncRegs();      // 刷新寄存器（上位机↔控制变量）
    Modbus_Poll();          // 处理已接收的Modbus帧
    Key_HandleEvent();      // 处理按键事件
    if (disp_flag)          // 每20ms刷新OLED
        OLED_Update();
    if (debug_flag)         // 每50ms串口打印调试信息
        Printf("Set:%d Spd:%d ...\n", ...);
}
```

主循环不做任何延时，全速运转。各功能靠中断里设的标志位控制执行频率。这种"超级循环+标志位"是嵌入式系统最经典的非 RTOS 架构。

### 8.6 OLED_Update — 屏幕显示

128×64 像素 OLED 分为 4 行（每行 16 像素高，对应 2 个 page）：

```
第1行(page0-1): SPD: +XXX rpm    [ON/OFF]    ← 当前转速 + 使能状态
第2行(page2-3): ANG:  XXX deg               ← 当前角度
第3行(page4-5): SET: +XXX rpm [SPD]         ← 当前模式的设定值
第4行(page6-7): S:+XXX    A: XXX            ← 按键调节目标（选中项反白）
                分隔线: row 15 / 31 / 47
```

采用全缓冲方式：先 `OLED_BuffClear()` 清空内存缓冲区，在缓冲区里画完所有内容，最后 `OLED_BuffShow()` 一次性通过 SPI DMA 刷到屏幕，避免闪烁。

---

## 九、上下位机协作时序总图

```
上位机(PC)                    RS485总线                 下位机(STC32G)
    │                            │                          │
    │   [写入设定值按钮]          │                          │
    │──→ FC=0x10 写regs[4~6] ──→│──→ UART2_ISR收字节       │
    │                            │    Modbus_RxByte()       │
    │                            │    ...2ms无新字节...      │
    │                            │    rx_done=1             │
    │                            │         │                │
    │                            │    主循环 Modbus_Poll()   │
    │                            │    ProcessFrame()        │
    │                            │    写入regs[4]=speed     │
    │                            │    写入regs[5]=angle     │
    │                            │    写入regs[6]=mode      │
    │   ←── FC=0x10 应答确认 ──←│←── SendResponse()        │
    │                            │         │                │
    │                            │    主循环 Modbus_SyncRegs()
    │                            │    g_set_speed=regs[4]   │
    │                            │    g_set_angle=regs[5]   │
    │                            │         │                │
    │                            │    Timer0中断(10ms)       │
    │                            │    Motor_ControlUpdate()  │
    │                            │    PID计算 → PWM输出      │
    │                            │    电机开始转动            │
    │                            │         │                │
    │   [自动轮询200ms]           │                          │
    │──→ FC=0x03 读regs[0~15] ─→│──→ 处理 → 应答16个寄存器  │
    │   ←── 应答含角度/速度等 ──←│                          │
    │   更新GUI显示               │                          │
```
