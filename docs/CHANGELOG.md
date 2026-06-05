# Changelog

## [0.3.0] - 2026-06-05

### Added — 力位混合夹爪控制器

#### 核心功能
- **GripperController** (`src/dm_motor/gripper.py`): 三模式夹爪控制器
  - 纯位置模式 (POSITION): 软件 PD + 最小急动度轨迹规划
  - 纯力矩模式 (TORQUE): 恒力矩输出
  - 力位混合模式 (HYBRID): 三状态机 (TRACKING → HOLDING → COMPLYING)
  - 自动堵转标定 (`calibrate_auto`): 找到开合极限 + set_zero 固化
  - 标定后支持百分比/毫米/弧度三种位置命令
  - 安全机制: 温度保护、反馈超时、力矩钳位、欠压错误自动重使能

#### SDK 增强
- **MotorType 枚举** (13 种达妙电机) + 每种电机的 PMAX/VMAX/TMAX 参数
- **CtrlMode 枚举**: MIT(1) / POS_VEL(2) / VEL(3) / POS_FORCE(4)
- 新控制方法: `control_pos_force()`, `control_pos_vel()`, `set_zero()`, `switch_ctrl_mode()`
- **底层驱动切换**: 从 libdm_device.so ctypes → usb_class.so Cython (兼容性更好)
- 自动检测平台架构加载对应 .so

#### 工具链
- `tools/diagnose_gripper.py`: 自动振荡诊断 (FFT + LPC + 阶跃响应分析)
- `tools/find_stable_kp.py`: Kp 稳定性边界扫描
- `tools/plot_tracking.py`: 轨迹跟踪可视化 (pos/vel/tau 三通道 + matplotlib PNG)
- `tools/tune_gripper.py`: 自动 Kp/Kd 调参 (阶跃响应迭代优化)

#### 第三方依赖
- `vendor/dm_device_sdk/`: 达妙 SDK v1.0.0 (libdm_device.so + usb_class.so)
- 自动检测 libusb 路径，无需手动配置

### 控制算法详解

#### 1. 最小急动度轨迹规划 (Minimum-Jerk Trajectory)

**问题**: 阶跃位置指令导致电机瞬间输出最大力矩，激发丝杠 stick-slip 振荡。

**算法**: 生成首尾加速度为零的平滑 S 曲线:
```
s(t) = 10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵
q_des(t) = q0 + (qf - q0) × s(t)
dq_des(t) = (qf - q0) × ds/dt
```

**作用**: 位置目标平滑过渡，力矩指令无突变，不激发机械共振。

**为什么选这个**: 相比梯形速度曲线，min-jerk 在起止点加速度为零，对丝杠机构最友好。

#### 2. 软件 PD + 硬件阻尼分离架构

**问题**: 电机内部 10kHz PD 的 Kp 项在丝杠上产生 stick-slip 极限环。任何 Kp≥10 都会振荡。

**算法**: 
- 电机 MIT 模式: `Kp=0, Kd>0` — 只做阻尼器，不做位置控制器
- 软件 200Hz 循环: `tau_ff = Kp_sw × (q_des - q_actual)` — 计算位置控制力矩

```
电机执行: τ = 0×(q_des-q) + Kd×(dq_des-dq) + tau_ff
         = Kd×(dq_des-dq) + Kp_sw×error    ← 位置控制在 tau_ff 里
```

**作用**: 避免电机内部 PD 与丝杠摩擦互相激发振荡。

**为什么**: 诊断数据 (find_stable_kp) 显示硬件 Kp≥10 必然振荡，但 Kp=0+tau_ff 完全稳定。

#### 3. dq_des 速度前馈

**问题**: 运动中 `Kd×(0 - dq_actual)` 产生阻力，与驱动力矩对抗引起力矩纹波。

**算法**: 轨迹跟踪时设 `dq_des = 轨迹速度`，使 `Kd×(dq_des - dq) ≈ 0`:
- 运动中: `dq_des = dq_trajectory` → Kd 不阻碍运动
- 到达后: `dq_des = 0` → Kd 提供全阻尼刹车

**作用**: 消除运动中 Kd 产生的阻力抖动。

#### 4. 运动/保持阻尼分离 (Kd 调度)

**问题**: 高 Kd 在高速运动时放大丝杠齿槽振动。

**算法**:
- 轨迹运动中: `Kd = 0.5` (低阻尼，不放大振动)
- 到达目标后: `Kd = 2.0` (高阻尼，快速稳定)

**为什么**: 波形分析 (plot_tracking) 显示运动中 Kd=2 把 ±3 rad/s 速度纹波放大为 ±0.5 Nm 力矩振荡。

#### 5. 力矩低通滤波 (EMA Filter)

**问题**: 200Hz 软件 PD 每 5ms 更新一次 tau_ff，离散力矩阶跃激发机械振动。

**算法**: 指数移动平均:
```
tau_filtered += α × (tau_raw - tau_filtered)    α=0.15, cutoff≈5Hz
```

**作用**: 平滑力矩指令，过滤 >5Hz 的力矩阶跃。

#### 6. 力位混合状态机

**算法**: 三状态 + 迟滞防振荡:
```
TRACKING → HOLDING:  |τ| ≥ force_limit
HOLDING → COMPLYING: |τ| ≥ 1.5 × force_limit
HOLDING → TRACKING:  |τ| < 0.8 × force_limit (迟滞)
COMPLYING → TRACKING: |τ| < 0.8 × force_limit
```

**为什么用迟滞**: 防止在力阈值边界反复切换导致振荡。

### 调试方法论

1. **数据驱动诊断**: 先采集再改，不猜测
   - `diagnose_gripper.py`: 7 组对照实验，FFT + LPC 自动检测振荡
   - `find_stable_kp.py`: Kp=1~15 逐个扫描，精确找到稳定边界
   - `plot_tracking.py`: pos/vel/tau 三通道波形可视化

2. **deep-research workflow**: 109 个 agent 并行搜索 6 个角度，验证 25 条论文声明
   - 确认 LPC 振荡检测方法 (Sharma 2020)
   - 确认 LuGre 摩擦补偿在丝杠上 40-63% 误差减少 (MDPI Machines 2022)
   - 确认 ESC 在丝杠上 1-3 分钟自动收敛 (IEEE TCST 2020)

### Changed
- `sdk.py`: 底层从 libdm_device.so ctypes 切换到 usb_class.so Cython 驱动
- `encode_mit/decode_feedback`: 支持可配置 PMAX/VMAX/TMAX
- `DmDevice.__init__()`: 接受 `motor_type` 参数

## [0.1.0] - 2026-05-29

### Added
- 初始版本: ctypes 包装 libdm_device.so
- MIT/VEL 模式控制、CAN/CAN-FD 学习笔记、故障排查文档
