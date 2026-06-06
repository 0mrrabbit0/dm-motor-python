# DM4310 丝杠夹爪控制算法技术报告

> **版本**: v0.3.0 | **日期**: 2026-06-05 | **硬件**: DM-J4310-2EC + 丝杠连杆夹爪  
> **概述文档**: [gripper_design.md](gripper_design.md) | **代码**: [src/dm_motor/gripper.py](../src/dm_motor/gripper.py)

---

## 目录

1. [系统概述](#1-系统概述)
2. [问题分析：丝杠振荡机理](#2-问题分析丝杠振荡机理)
3. [控制架构选型](#3-控制架构选型)
4. [算法 1：最小急动度轨迹规划](#4-算法-1最小急动度轨迹规划)
5. [算法 2：软件 PD + 硬件阻尼分离](#5-算法-2软件-pd--硬件阻尼分离)
6. [算法 3：dq_des 速度前馈](#6-算法-3dq_des-速度前馈)
7. [算法 4：Kd 调度策略](#7-算法-4kd-调度策略)
8. [算法 5：力矩 EMA 低通滤波](#8-算法-5力矩-ema-低通滤波)
9. [算法 6：力位混合状态机](#9-算法-6力位混合状态机)
10. [标定系统](#10-标定系统)
11. [安全机制](#11-安全机制)
12. [实验结果](#12-实验结果)
13. [参数速查表](#13-参数速查表)
14. [未来工作：LuGre 摩擦补偿路线图](#14-未来工作lugre-摩擦补偿路线图)

---

## 1. 系统概述

### 1.1 机械结构

```
DM-J4310 电机 → 丝杠 → 套筒 → 连杆 → 两指夹爪
     θ(rad)      线性位移         开合距离 d(mm)
```

电机旋转角度 θ 通过丝杠转换为套筒线性位移，再由连杆机构转换为夹爪开合距离。整个传动链存在显著的库仑摩擦和 Stribeck 效应。

### 1.2 电机参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 型号 | DM-J4310-2EC | 准直驱无刷电机 |
| 额定力矩 | 10 Nm (TMAX) | MIT 编码范围 |
| 最大转速 | 30 rad/s (VMAX) | MIT 编码范围 |
| 位置范围 | ±12.5 rad (PMAX) | MIT 编码范围 |
| 控制模式 | MIT (CTRL_MODE=1) | 阻抗控制 |
| 内部控制频率 | ~10 kHz | FOC + PD 环 |
| 通信 | CAN-FD 5Mbps | 1M/5M 仲裁/数据 |

### 1.3 MIT 阻抗控制方程

电机内部以 ~10kHz 频率执行：

$$\tau_{motor} = K_p \cdot (q_{des} - q) + K_d \cdot (\dot{q}_{des} - \dot{q}) + \tau_{ff}$$

软件每个控制周期可设定的 **5 个参数**：

| 参数 | 符号 | 范围 | 量化 | 说明 |
|------|------|------|------|------|
| 位置增益 | $K_p$ | 0 ~ 500 | 12-bit | 位置刚度 (Nm/rad) |
| 速度增益 | $K_d$ | 0 ~ 5 | 12-bit | 速度阻尼 (Nm·s/rad) |
| 目标位置 | $q_{des}$ | ±PMAX | 16-bit | 目标角度 (rad) |
| 目标速度 | $\dot{q}_{des}$ | ±VMAX | 12-bit | 目标角速度 (rad/s) |
| 前馈力矩 | $\tau_{ff}$ | ±TMAX | 12-bit | 附加力矩 (Nm) |

### 1.4 控制目标

三种运行模式：

| 模式 | 行为 | 应用场景 |
|------|------|---------|
| **POSITION** | 移动到目标位置，忽略外力 | 打开/闭合夹爪 |
| **TORQUE** | 输出恒定力矩，不管位置 | 恒力夹持 |
| **HYBRID** | 向目标运动，遇力停住/柔顺跟随 | 安全夹取易碎物体 |

### 1.5 约束条件

- 软件控制频率：**200 Hz**（受 CAN-FD USB 桥 + Python 线程调度限制）
- 电机内部 PD 频率：**~10 kHz**
- 只能通过上述 5 个参数影响电机行为
- 丝杠传动链存在显著非线性摩擦

---

## 2. 问题分析：丝杠振荡机理

### 2.1 Stribeck 负阻尼效应

丝杠机构的摩擦力与速度的关系不是简单的线性关系，而是呈现 Stribeck 曲线：

```
摩擦力
  Fs ┤ ╲
     │   ╲         ╱────── 粘性摩擦区 (线性)
  Fc ┤    ╲───────╱
     │     Stribeck
     │     负斜率区
     ├─────┬──────┬──────→ 速度
     0    vs    v_high
```

在 Stribeck 区间（0 < v < vs），摩擦力随速度增大反而减小。这等效于一个**负阻尼**：

$$F_{friction}(v) = F_c + (F_s - F_c) \cdot e^{-(v/v_s)^2} + F_v \cdot v$$

对此摩擦力关于速度求导：

$$\frac{\partial F}{\partial v} = -\frac{2(F_s - F_c)}{v_s^2} \cdot v \cdot e^{-(v/v_s)^2} + F_v$$

当 $v$ 较小时，第一项为负且绝对值大于 $F_v$，系统等效总阻尼为负，**任何位置反馈环都会被激发为自持振荡（极限环）**。

### 2.2 硬件 PD 与丝杠摩擦的耦合

当使用电机内部 PD（$K_p > 0$），系统动力学为：

$$J\ddot{q} = K_p(q_{des} - q) + K_d(\dot{q}_{des} - \dot{q}) - F_{friction}(\dot{q})$$

线性化后特征方程：

$$Js^2 + (K_d + F_v - \frac{\partial F_{Stribeck}}{\partial v})s + K_p = 0$$

**稳定性条件**（Routh-Hurwitz）：

$$K_d + F_v > \frac{\partial F_{Stribeck}}{\partial v}\bigg|_{v \approx 0}$$

即要求总阻尼大于 Stribeck 负阻尼。对于丝杠系统，$\frac{\partial F_{Stribeck}}{\partial v}$ 的绝对值可能很大（摩擦力在零速附近变化剧烈），导致：

- **$K_p$ 越大 → 系统固有频率越高 → 经过 Stribeck 区的频率越高 → 越容易激发振荡**
- **$K_d$ 有限（最大 5 Nm·s/rad）→ 无法提供足够阻尼**

### 2.3 实验验证

使用 `tools/diagnose_gripper.py` 进行 7 组对照实验（每组 3 秒，200Hz 采样）：

| 测试条件 | pos p2p (rad) | vel RMS (rad/s) | FFT 峰值 (Hz) | LPC 检测 | 结论 |
|----------|--------------|----------------|---------------|----------|------|
| Kp=0, Kd=2 | 0.0000 | 0.013 | — | 稳定 | **基准** |
| Kp=3, Kd=2 | 0.0000 | 0.013 | — | 稳定 | 安全区 |
| **Kp=10, Kd=3** | **0.0603** | **2.549** | 0.33 | **振荡** | **极限环** |
| **Kp=20, Kd=2** | **0.0553** | **1.877** | 52.3 | **振荡** | **极限环** |
| 阶跃 Kp=10 | 0.8213 | 2.785 | 0.33 | 振荡 | 发散 |
| 恒力矩 τ=0.3 | 0.0004 | 0.012 | — | 稳定 | 无 PD 无振荡 |
| **Kp=10@200Hz** | **0.1785** | **2.867** | 0.33 | **振荡** | 频率无关 |

**关键发现**：
1. $K_p = 0$：绝对稳定（pos p2p = 0）
2. $K_p = 3$：稳定
3. $K_p \geq 10$：必然振荡，与发送频率无关
4. 纯力矩模式：稳定 — 确认问题在位置反馈环

### 2.4 Kp 稳定性边界

使用 `tools/find_stable_kp.py` 扫描 Kp = 1 ~ 15（$K_d = 2.0$，200Hz）：

```
   Kp |      p2p |      std | vel_chg/s | Result
-------------------------------------------------------
    1 |  0.00000 |  0.00000 |       0.0 |   STABLE
    3 |  0.00038 |  0.00003 |       1.7 |   STABLE
    5 |  0.00038 |  0.00003 |       1.7 |   STABLE
    8 |  0.00000 |  0.00000 |       0.0 |   STABLE
   10 |  0.00038 |  0.00003 |       1.7 |   STABLE
   15 |  0.00038 |  0.00002 |       0.8 |   STABLE
```

> **注意**：此结果在电机重启后立即测试，初始状态良好。在运动后测试（丝杠预紧状态不同），Kp≥10 会振荡。说明丝杠摩擦状态是时变的，安全的 Kp 边界取决于运行历史。

**结论**：不能依赖硬件 Kp 做位置控制。需要从架构层面绕过这个问题。

---

## 3. 控制架构选型

### 3.1 候选方案对比

| 方案 | 原理 | 优点 | 缺点 | 适用性 |
|------|------|------|------|--------|
| **A: 纯硬件 PD** | $K_p > 0, K_d > 0$, 直接设 $q_{des}$ | 简单，10kHz 带宽 | Kp≥10 振荡，丝杠不可用 | **不适用** |
| **B: POS_FORCE 模式** | CTRL_MODE=4，硬件电流限幅 | 硬件级力限制 | 需切换模式，Kp 仍在硬件 | 不推荐 |
| **C: 纯软件力矩** | $K_p=0, K_d=0$, 全靠 $\tau_{ff}$ | 完全可控 | 200Hz 无阻尼，抗扰差 | 不推荐 |
| **D: 软件 PD + 硬件阻尼** | $K_p=0$, $K_d>0$, $\tau_{ff}$ 做位控 | 10kHz 阻尼 + 200Hz 位控 | 需要轨迹规划 | **选用** |

### 3.2 选型依据

选择方案 D 的核心逻辑：

1. **$K_p = 0$ 消除 Stribeck 振荡的根源** — 没有位置反馈环，就不会与丝杠摩擦耦合
2. **$K_d > 0$ 保留 10kHz 高带宽阻尼** — 电机内部快速响应速度扰动，这是 200Hz 软件环做不到的
3. **$\tau_{ff}$ 做位置控制** — 软件计算 $\tau_{ff} = K_{p,sw} \cdot (q_{des} - q_{actual})$，等效于一个 200Hz PD 控制器，但力矩输出经过滤波后不会有高频分量激发 Stribeck 效应

### 3.3 系统框图

```
                        ┌─── 200Hz 软件环 ───┐        ┌─── 10kHz 电机内环 ───┐
                        │                    │        │                       │
 q_target ──→ [轨迹规划] ──→ q_des            │        │                       │
                │            │               │        │                       │
                └──→ dq_des  │               │        │                       │
                             ▼               │        │                       │
              ┌─── [软件PD] ◄── q_actual ◄───┼────────┤  q_actual             │
              │     Kp_sw·e                  │        │    ▲                  │
              ▼               ┌──────┐       │        │    │                  │
         tau_raw ──→ [EMA] ──→│tau_ff│──────►│─►MIT──►│───►[电机+丝杠] ──►q   │
                     滤波器    └──────┘       │  Kp=0  │    ▲              │   │
                              ┌──────┐       │  Kd>0  │    │              │   │
              dq_des ────────►│dq_des│──────►│────────┤    └──────────────┘   │
                              └──────┘       │        │ Kd·(dq_des - dq)     │
                        └────────────────────┘        └───────────────────────┘
```

**信号流**：
1. 轨迹规划生成平滑的 $(q_{des}, \dot{q}_{des})$
2. 软件 PD 计算 $\tau_{raw} = K_{p,sw} \cdot (q_{des} - q_{actual})$
3. EMA 滤波器平滑力矩：$\tau_{ff} = \text{EMA}(\tau_{raw})$
4. 电机执行：$\tau_{motor} = K_d \cdot (\dot{q}_{des} - \dot{q}) + \tau_{ff}$

---

## 4. 算法 1：最小急动度轨迹规划

### 4.1 数学推导

**目标**：找到从 $(q_0, 0, 0)$ 到 $(q_f, 0, 0)$ 的轨迹（位置、速度、加速度边界条件），使急动度（jerk = 加速度的导数）的积分最小：

$$\min_{q(t)} \int_0^T \overset{\cdots}{q}(t)^2 \, dt$$

**边界条件（6 个）**：
$$q(0) = q_0, \quad \dot{q}(0) = 0, \quad \ddot{q}(0) = 0$$
$$q(T) = q_f, \quad \dot{q}(T) = 0, \quad \ddot{q}(T) = 0$$

由变分法（Euler-Lagrange 方程 $\frac{d^6 q}{dt^6} = 0$），解为 5 次多项式。代入边界条件后：

$$q(t) = q_0 + (q_f - q_0) \cdot s(\sigma)$$

其中 $\sigma = t/T$，归一化相位函数：

$$s(\sigma) = 10\sigma^3 - 15\sigma^4 + 6\sigma^5$$

### 4.2 各阶导数

| 阶 | 表达式 | 端点值 |
|----|--------|--------|
| 位置 | $s(\sigma) = 10\sigma^3 - 15\sigma^4 + 6\sigma^5$ | $s(0)=0, s(1)=1$ |
| 速度 | $\dot{s} = \frac{1}{T}(30\sigma^2 - 60\sigma^3 + 30\sigma^4)$ | $\dot{s}(0)=0, \dot{s}(1)=0$ |
| 加速度 | $\ddot{s} = \frac{1}{T^2}(60\sigma - 180\sigma^2 + 120\sigma^3)$ | $\ddot{s}(0)=0, \ddot{s}(1)=0$ |
| 急动度 | $\overset{\cdots}{s} = \frac{1}{T^3}(60 - 360\sigma + 360\sigma^2)$ | $\overset{\cdots}{s}(0)=60/T^3$ (有限值) |

**峰值速度**出现在 $\sigma = 0.5$：

$$\dot{q}_{peak} = \frac{q_f - q_0}{T} \cdot \dot{s}(0.5) = \frac{q_f - q_0}{T} \cdot 1.875$$

### 4.3 为什么选最小急动度

| 轨迹类型 | 加速度连续 | 零端点加速度 | 最优平滑 | 适合丝杠 |
|----------|-----------|-------------|---------|---------|
| 梯形速度 | 不连续 | 否 | 差 | 加速度跳变激发振动 |
| S 曲线 (7 段) | 连续 | 否 | 中 | 参数多，调试复杂 |
| 3 次多项式 | 不连续 | 否 | 差 | 速度端点可控，加速度不可控 |
| **5 次 min-jerk** | **连续** | **是** | **最优** | **零加速度起止，力矩无突变** |

### 4.4 自适应时长计算

轨迹时长 $T$ 按距离比例缩放：

$$T = T_{full} \cdot \frac{|q_{target} - q_{current}|}{|q_{open} - q_{close}|}$$

其中 $T_{full} = 2.5$ s 为全行程基准时长，下限钳位 0.1 s。

**峰值速度控制**：全行程 ~11 rad，$T = 2.5$ s → $v_{peak} = 11 \times 1.875 / 2.5 = 8.25$ rad/s。实验验证此速度下位置跟踪平滑。

### 4.5 代码映射

```python
# src/dm_motor/gripper.py: MinJerkTrajectory 类
class MinJerkTrajectory:
    def sample(self) -> tuple[float, float, bool]:
        s = t / self.duration
        phase = 10 * s**3 - 15 * s**4 + 6 * s**5          # 位置相位
        dphase = (30 * s**2 - 60 * s**3 + 30 * s**4) / T   # 速度相位
        q_des = q0 + (qf - q0) * phase
        dq_des = (qf - q0) * dphase
        return q_des, dq_des, done
```

---

## 5. 算法 2：软件 PD + 硬件阻尼分离

### 5.1 MIT 阻抗控制方程展开

电机内部执行的完整方程：

$$\tau_{motor} = \underbrace{K_p \cdot (q_{des} - q)}_{\text{位置项}} + \underbrace{K_d \cdot (\dot{q}_{des} - \dot{q})}_{\text{速度项}} + \underbrace{\tau_{ff}}_{\text{前馈项}}$$

### 5.2 设定 Kp=0 后的等效系统

令 $K_p = 0$，软件计算 $\tau_{ff} = K_{p,sw} \cdot (q_{des,traj} - q)$：

$$\tau_{motor} = K_d \cdot (\dot{q}_{des} - \dot{q}) + K_{p,sw} \cdot (q_{des,traj} - q)$$

系统动力学（忽略摩擦）：

$$J\ddot{q} + K_d\dot{q} + K_{p,sw} \cdot q = K_d \cdot \dot{q}_{des} + K_{p,sw} \cdot q_{des,traj}$$

这是一个经典的二阶系统，等效固有频率和阻尼比：

$$\omega_n = \sqrt{\frac{K_{p,sw}}{J}}, \quad \zeta = \frac{K_d}{2\sqrt{J \cdot K_{p,sw}}}$$

### 5.3 关键区别：为什么软件 PD 不振荡

**硬件 PD（Kp>0 在 MIT 中）**：
- 10kHz 更新率 → 力矩响应极快
- 微小位置偏差立即产生力矩 → 克服静摩擦 → 位置跳变 → 反向力矩 → stick-slip

**软件 PD（Kp=0 in MIT, tau_ff 做位控）**：
- 200Hz 更新率 → 力矩变化平缓
- EMA 滤波后力矩更加平滑
- 不会产生高频力矩突变 → 不激发 Stribeck 负阻尼区
- 代价：位置跟踪带宽降至 ~5Hz（对夹爪够用）

### 5.4 闭环传递函数

软件环的离散传递函数（采样 $T_s = 5$ ms）：

$$G_{sw}(z) = \frac{K_{p,sw} \cdot H_{EMA}(z) \cdot G_{motor}(z)}{1 + K_{p,sw} \cdot H_{EMA}(z) \cdot G_{motor}(z)}$$

其中 EMA 滤波器：

$$H_{EMA}(z) = \frac{\alpha}{1 - (1-\alpha)z^{-1}}$$

电机（简化为 $K_d$ 阻尼 + 惯量 $J$）的离散模型对 200Hz 带宽内近似一阶：

$$G_{motor}(z) \approx \frac{1}{K_d} \cdot \frac{1 - e^{-K_d T_s / J}}{1 - e^{-K_d T_s / J} z^{-1}}$$

**稳定性**：由于 EMA 滤波器（$\alpha = 0.15$）严格限制了环路增益的带宽在 ~5Hz 以下，远低于丝杠的 Stribeck 共振频率（通常 20-50Hz），系统有充足的相位裕度。

### 5.5 代码映射

```python
# _step_position (gripper.py L546-567)
error = q_des - self._pos                    # 位置误差
tau_raw = self.kp_track * error              # 软件 P 项
tau_raw = clamp(tau_raw, ±tau_safety)        # 力矩钳位
tau_cmd = self._filter_tau(tau_raw)          # EMA 低通滤波
self._send_mit(Kp=0, Kd=kd, dq=dq_des, tau=tau_cmd)  # 下发
```

---

## 6. 算法 3：dq_des 速度前馈

### 6.1 问题：Kd 阻力项

MIT 方程中速度项展开：

$$\tau_{velocity} = K_d \cdot (\dot{q}_{des} - \dot{q})$$

若 $\dot{q}_{des} = 0$（不设速度前馈）：

$$\tau_{velocity} = -K_d \cdot \dot{q}$$

在运动过程中 $\dot{q} \neq 0$，此项产生**反向阻力**。例如 $K_d = 2, \dot{q} = 8$ rad/s 时，阻力为 $-16$ Nm（已超过 TMAX），实际被钳位到 $-10$ Nm。这个阻力与软件 PD 的驱动力矩对抗，导致力矩纹波。

### 6.2 前馈抵消原理

设 $\dot{q}_{des} = \dot{q}_{trajectory}$（轨迹规划的速度），则：

$$\tau_{velocity} = K_d \cdot (\dot{q}_{trajectory} - \dot{q}_{actual})$$

当跟踪良好时 $\dot{q}_{actual} \approx \dot{q}_{trajectory}$，因此：

$$\tau_{velocity} \approx 0$$

**Kd 项从"持续阻力"变为"偏差修正"** — 只在实际速度偏离轨迹时才起作用。

### 6.3 量化误差分析

MIT 协议中 $\dot{q}_{des}$ 编码为 12-bit，范围 $\pm 30$ rad/s：

$$\Delta \dot{q} = \frac{60}{4095} = 0.01465 \text{ rad/s}$$

量化引起的力矩误差：

$$\Delta \tau = K_d \cdot \Delta \dot{q} = 2.0 \times 0.01465 = 0.029 \text{ Nm}$$

对比实验中的力矩纹波 ~0.03 Nm（优化后），量化误差是纹波的主要来源之一，但已在可接受范围内。

### 6.4 代码映射

```python
# 轨迹跟踪时：
q_des, dq_des, done = traj.sample()           # 轨迹给出 dq_des
self._send_mit(0, kd, dq=dq_des, tau=tau_cmd) # dq_des 传入 MIT

# 到达后（dq_des=0）：
self._send_mit(0, kd, dq=0.0, tau=tau_cmd)    # Kd 提供全阻尼
```

---

## 7. 算法 4：Kd 调度策略

### 7.1 运动阶段 vs 保持阶段

| 阶段 | 需求 | Kd 值 | 理由 |
|------|------|-------|------|
| **轨迹跟踪** | 平滑运动 | **0.5** | 高 Kd 放大丝杠齿槽振动 |
| **到达保持** | 快速稳定 | **2.0** | 需要强阻尼消除残余振动 |

### 7.2 波形对比

**优化前**（运动全程 Kd=2.0）：
- 速度：±3-5 rad/s 高频振荡
- 力矩：±0.5 Nm 纹波

**优化后**（运动 Kd=0.5，保持 Kd=2.0）：
- 速度：平滑钟形曲线，纹波 <0.5 rad/s
- 力矩：纹波 <0.05 Nm

### 7.3 切换时机

```
运动开始 ──── Kd=0.5 ──── 轨迹结束(done=True) ──── Kd=2.0 ──── 保持
```

切换在轨迹 `done` 标志翻转时发生（不是渐变）。由于 $\dot{q}_{des}$ 在轨迹末尾已趋近 0（min-jerk 特性），Kd 突变不会产生力矩跳变：

$$\Delta \tau_{Kd} = (K_{d,new} - K_{d,old}) \cdot (0 - \dot{q}|_{t=T}) \approx 0$$

### 7.4 代码映射

```python
# _step_position (gripper.py L548-557)
if traj and not self._traj_done:
    ...
    if done:
        kd = self.kd_track       # 2.0: 保持阶段高阻尼
    else:
        kd = 0.5                 # 运动阶段低阻尼
else:
    kd = self.kd_track           # 已在保持阶段
```

---

## 8. 算法 5：力矩 EMA 低通滤波

### 8.1 离散 EMA 推导

指数移动平均（Exponential Moving Average）递推公式：

$$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]$$

Z 变换：

$$H(z) = \frac{\alpha}{1 - (1-\alpha)z^{-1}}$$

### 8.2 α 与截止频率的关系

EMA 的 -3dB 截止频率：

$$f_c = \frac{f_s}{2\pi} \cdot \arccos\left(\frac{2 - \alpha - \alpha\sqrt{4/\alpha - 3}}{2(1-\alpha)}\right)$$

近似公式（$\alpha \ll 1$ 时）：

$$f_c \approx \frac{\alpha \cdot f_s}{2\pi(1-\alpha)}$$

### 8.3 参数选择

| 参数 | 值 | 推导 |
|------|-----|------|
| 采样率 $f_s$ | 200 Hz | 控制循环频率 |
| $\alpha$ | 0.15 | 选定 |
| 截止频率 $f_c$ | **5.6 Hz** | $\frac{0.15 \times 200}{2\pi \times 0.85} = 5.62$ Hz |

**为什么 5.6 Hz**：
- 丝杠 Stribeck 振荡频率：20-50 Hz → 被完全滤除
- min-jerk 轨迹最大有效频率：$\sim 1/T \approx 0.4-2$ Hz → 完整通过
- 余量：5.6 Hz >> 2 Hz（轨迹） + 足够的相位裕度

### 8.4 滤波器引入的延迟

EMA 的群延迟（在低频近似）：

$$\tau_{delay} \approx \frac{1-\alpha}{\alpha \cdot f_s} = \frac{0.85}{0.15 \times 200} = 28.3 \text{ ms}$$

28 ms 延迟对于 2.5 秒轨迹（占 1.1%）可以忽略。对力位混合模式的力检测响应时间影响约 1-2 个控制周期（5-10 ms），在可接受范围。

### 8.5 代码映射

```python
# _filter_tau (gripper.py L530-534)
def _filter_tau(self, tau_raw, alpha=0.15):
    self._tau_cmd_filtered += alpha * (tau_raw - self._tau_cmd_filtered)
    return self._tau_cmd_filtered
```

---

## 9. 算法 6：力位混合状态机

### 9.1 状态转移图

```
                ┌───────────────────────────────────────────┐
                │                                           │
                ▼                                           │
    ┌───────────┐  |τ| ≥ F_limit   ┌───────────┐           │
    │ TRACKING  │ ────────────────→ │  HOLDING  │           │
    │ Kp_sw=15  │                   │  Kp_hw=2  │           │
    │ Kd=0.5    │                   │  Kd_hw=1.5│           │
    │ q=轨迹    │                   │  q=冻结   │           │
    └───────────┘ ←──────────────── └───────────┘           │
          │       |τ| < 0.8·F_limit    │                    │
          │       (迟滞)                │ |τ| ≥ 1.5·F_limit │
          │                            ▼                    │
          │                       ┌───────────┐             │
          │                       │ COMPLYING │             │
          └─────────────────────  │  Kp=0     │ ────────────┘
            |τ| < 0.8·F_limit    │  Kd=2.0   │   |τ| < F_limit
                                  └───────────┘
```

### 9.2 迟滞防振荡分析

状态转移使用**施密特触发器**逻辑：

- **进入阈值**：$|\tau| \geq F_{limit}$
- **退出阈值**：$|\tau| < 0.8 \cdot F_{limit}$
- **迟滞带宽**：$0.2 \cdot F_{limit}$

如果没有迟滞，在 $|\tau| \approx F_{limit}$ 时：
```
TRACKING → 力矩达到阈值 → HOLDING → Kp 降低 → 力矩下降 → TRACKING → 力矩上升 → HOLDING → ...
```
形成高频切换振荡。迟滞确保：
- 必须力矩**降到 80%** 才恢复跟踪
- 消除阈值附近的抖振（chattering）

### 9.3 各状态的 MIT 参数配置

| 状态 | $K_p$ (MIT) | $K_d$ (MIT) | $q_{des}$ | $\dot{q}_{des}$ | $\tau_{ff}$ | 效果 |
|------|------------|------------|-----------|----------------|------------|------|
| TRACKING | 0 | 0.5/2.0 | — | 轨迹速度 | 软件PD输出 | 跟踪目标 |
| HOLDING | 2.0 | 1.5 | 冻结位置 | 0 | 0 | 低刚度保持 |
| COMPLYING | 0 | 2.0 | — | 0 | 0 | 纯阻尼跟随 |

**HOLDING 用硬件 Kp=2**：保持阶段位置误差小（冻结在接触位置），Kp=2 不会触发 Stribeck 振荡（实验验证 Kp≤5 安全），同时提供柔软的弹性保持。

### 9.4 代码映射

```python
# _step_hybrid (gripper.py L573-627)
if state == TRACKING:
    if |tau| >= force_limit:
        hold_angle = current_pos         # 冻结位置
        state = HOLDING
    else:
        # 轨迹跟踪（同 _step_position）

elif state == HOLDING:
    if |tau| >= 1.5 * force_limit:       # 外力过大
        state = COMPLYING                # 释放
    elif |tau| < 0.8 * force_limit:      # 外力消失
        state = TRACKING                 # 恢复跟踪
    else:
        send_mit(Kp=2, Kd=1.5, q=hold)  # 柔性保持

elif state == COMPLYING:
    if |tau| < 0.8 * force_limit:
        state = TRACKING                 # 恢复跟踪
    else:
        send_mit(Kp=0, Kd=2)            # 纯阻尼跟随
```

---

## 10. 标定系统

### 10.1 自动堵转检测

标定流程自动找到两个机械极限：

```python
def _drive_until_stall(tau_cmd, label):
    # 每 0.1s 检测位置变化
    if |pos - last_pos| < 0.01 rad for 0.3s:
        return pos  # 位置不变 = 堵转

    # 欠压/过流错误也视为到达极限
    if motor_err not in (0, 1):
        return pos  # 电机保护触发 = 到达极限
```

**两种堵转判据**：
1. **位置不变**：$|\Delta q| < 0.01$ rad 持续 0.3s → 摩擦力大于驱动力矩
2. **电机保护**：err=9（欠压）或 err=10（过流）→ 堵转电流导致电源压降

### 10.2 set_zero 编码器归零

DM4310 使用增量编码器，每次上电从 0 开始计数。标定后调用 `set_zero`：

```
标定前：close = +12.35 rad, open = +1.30 rad （取决于上电位置）
                    ↓ set_zero(闭合位置)
标定后：close = 0.0 rad, open = -11.05 rad   （固定值）
```

标定数据保存到 `gripper_calibration.json`，后续启动直接加载。

### 10.3 坐标映射

三种位置表示：

$$\text{开合百分比} = \frac{\theta - \theta_{close}}{\theta_{open} - \theta_{close}} \times 100\%$$

$$\text{开合距离(mm)} = \frac{\theta - \theta_{close}}{\theta_{open} - \theta_{close}} \times d_{stroke}$$

其中 $d_{stroke}$ 为用户测量的全开口距离（可选）。

---

## 11. 安全机制

### 11.1 保护项清单

| 保护项 | 阈值 | 响应 | 代码位置 |
|--------|------|------|---------|
| MOS 管温度 | > 80°C | 立即 disable | `_check_safety` |
| 转子温度 | > 100°C | 立即 disable | `_check_safety` |
| 电机错误码 | ≠ 0 或 1 | 立即 disable | `_check_safety` |
| 反馈超时 | > 500ms | 立即 disable | `_control_loop` |
| 力矩钳位 | ±8 Nm | 软件限幅 | `_step_position` |
| 欠压自动重使能 | err=0 | 重新 enable | `_control_loop` |
| 位置钳位 | close~open 范围 | 命令限幅 | `_clamp_angle` |

### 11.2 启动安全序列

```
enable() → 5次发送0xFC → 等待100ms → 100周期纯阻尼(Kp=0,Kd=2) → 读取初始位置 → 启动控制循环
```

**纯阻尼启动**防止使能瞬间因残余位置偏差产生冲击力矩。

---

## 12. 实验结果

### 12.1 振荡诊断对照表

7 组测试（`tools/diagnose_gripper.py`），每组 3 秒 200Hz 采样：

| # | 条件 | pos p2p (rad) | vel RMS (rad/s) | 振荡 |
|---|------|--------------|----------------|------|
| 1 | Kp=0, Kd=2 (纯阻尼) | 0.000 | 0.013 | 否 |
| 2 | Kp=3, Kd=2 | 0.000 | 0.013 | 否 |
| 3 | Kp=10, Kd=3 | **0.060** | **2.549** | **是** |
| 4 | Kp=20, Kd=2 | **0.055** | **1.877** | **是** |
| 5 | 阶跃 Kp=10 | **0.821** | **2.785** | **是** |
| 6 | 恒力矩 τ=0.3 | 0.000 | 0.012 | 否 |
| 7 | Kp=10@200Hz | **0.179** | **2.867** | **是** |

### 12.2 轨迹跟踪波形

使用 `tools/plot_tracking.py` 采集（最终版本，含所有优化）：

- **小行程 40%→60%**：位置和速度完全平滑，力矩纹波 <0.05 Nm
- **大行程 10%→90%**：位置平滑，速度轻微纹波 <0.5 rad/s，物理无感振动
- **全行程 0%→100%**：位置跟踪精确，力矩纹波 ~0.03 Nm（丝杠机械噪声）
- **反向 90%→10%**：对称特性，与正向一致

### 12.3 优化迭代对比

| 版本 | 位置 | 速度振幅 | 力矩振幅 | 体感 |
|------|------|---------|---------|------|
| v1: 硬件 Kp=20 | 振荡 | ±5 rad/s | ±1.0 Nm | 剧烈抖动 |
| v2: 硬件 Kp=10 + 轨迹 | 跟踪 | ±5 rad/s | ±1.0 Nm | 全程抖 |
| v3: 软件PD + 轨迹 | 跟踪 | ±3 rad/s | ±0.5 Nm | 有抖 |
| v4: +dq_des 前馈 | 跟踪 | ±3 rad/s | ±0.5 Nm | 轻微 |
| **v5: +Kd调度+EMA** | **平滑** | **<0.5 rad/s** | **<0.05 Nm** | **无感** |

---

## 13. 参数速查表

### 控制参数

| 参数 | 默认值 | 范围 | 作用 | 调节建议 |
|------|--------|------|------|---------|
| `kp_track` | 15.0 | 5-50 | 软件 PD 位置增益 (Nm/rad) | 越大跟踪越快，太大不稳定 |
| `kd_track` | 2.0 | 0.5-5.0 | 保持阶段阻尼 (Nm·s/rad) | 越大稳定越快，太大过阻尼 |
| 运动 Kd | 0.5 | 0.1-1.0 | 运动阶段阻尼 | 越小越顺滑，太小可能超调 |
| `kp_hold` | 2.0 | 1-5 | 力位混合保持刚度 | 越大保持越硬 |
| `kd_hold` | 1.5 | 0.5-3.0 | 力位混合保持阻尼 | — |
| `kd_comply` | 2.0 | 1.0-4.0 | 柔顺模式阻尼 | 越大跟随越缓 |
| `tau_safety` | 8.0 | 1-10 | 力矩钳位上限 (Nm) | 保护机构 |
| `move_duration` | 2.5 | 0.5-5.0 | 全行程基准时长 (s) | 越短越快，太短可能振荡 |
| `hysteresis` | 0.8 | 0.6-0.95 | 迟滞比 | 越小响应越灵敏 |

### EMA 滤波参数

| 参数 | 值 | 截止频率 |
|------|-----|---------|
| α = 0.10 | 保守 | 3.5 Hz |
| **α = 0.15** | **默认** | **5.6 Hz** |
| α = 0.25 | 激进 | 10.6 Hz |

### MIT 下发参数一览

| 模式·阶段 | Kp | Kd | q_des | dq_des | tau_ff |
|-----------|----|----|-------|--------|--------|
| POSITION·运动 | 0 | 0.5 | — | 轨迹速度 | 软件PD |
| POSITION·保持 | 0 | 2.0 | — | 0 | 软件PD |
| TORQUE | 0 | 0.5 | — | 0 | 用户力矩 |
| HYBRID·跟踪 | 0 | 0.5 | — | 轨迹速度 | 软件PD |
| HYBRID·保持 | 2.0 | 1.5 | 冻结位置 | 0 | 0 |
| HYBRID·柔顺 | 0 | 2.0 | — | 0 | 0 |

---

## 14. 未来工作：LuGre 摩擦补偿路线图

### 14.1 目标

消除残余力矩纹波（~0.03 Nm），并提升开合速度（从 2.5s 降至 <1s）。

### 14.2 LuGre 摩擦模型

$$F_{friction} = \sigma_0 z + \sigma_1 \dot{z} + F_v \cdot v$$

$$\dot{z} = v - \sigma_0 \frac{|v|}{g(v)} z$$

$$g(v) = F_c + (F_s - F_c) e^{-(v/v_s)^2}$$

需辨识的参数：$\sigma_0, \sigma_1, F_c, F_s, v_s, F_v$

### 14.3 实施步骤

1. **摩擦辨识**：用恒速/变速测试提取 Stribeck 曲线参数
2. **观测器设计**：估计不可测量的鬃毛变形 $z$
3. **前馈补偿**：$\tau_{ff} = K_{p,sw} \cdot error + \hat{F}_{friction}$
4. **提速验证**：补偿后降低 `move_duration` 到 <1s

### 14.4 预期效果

根据文献（MDPI Machines 2022），LuGre 补偿可将丝杠跟踪误差减少 40-63%，使更高速度下的运动仍然平滑。
