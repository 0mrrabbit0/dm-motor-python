# 夹爪力位混合控制 — 设计概述

> **详细算法报告**：[control_report.md](control_report.md)（含数学推导、稳定性分析、实验数据）

## 系统结构

```
DM-J4310 电机 → 丝杠 → 套筒 → 连杆 → 两指夹爪
```

## 三种控制模式

| 模式 | 行为 | API |
|------|------|-----|
| POSITION | 移动到目标位置（mm/%%/rad）| `move_to_pct(50)` |
| TORQUE | 恒力矩输出 | `set_torque(2.0)` |
| HYBRID | 力限位控（遇力停/柔顺）| `grip_pct(0, force_limit=3.0)` |

## 核心控制架构

**软件 PD + 硬件阻尼分离**：电机内部 Kp=0（不做位置控制），Kd>0（提供 10kHz 阻尼）。位置控制通过软件计算力矩经 tau_ff 下发。

解决了丝杠系统 Stribeck 负阻尼导致的 stick-slip 振荡。详见 [control_report.md 第 2-5 章](control_report.md#2-问题分析丝杠振荡机理)。

## 使用的算法

1. **最小急动度轨迹规划** — 5次多项式 S 曲线 → [详述](control_report.md#4-算法-1最小急动度轨迹规划)
2. **软件 PD via tau_ff** — 绕过硬件 PD 振荡 → [详述](control_report.md#5-算法-2软件-pd--硬件阻尼分离)
3. **dq_des 速度前馈** — 抵消 Kd 运动阻力 → [详述](control_report.md#6-算法-3dq_des-速度前馈)
4. **Kd 调度** — 运动/保持双阻尼 → [详述](control_report.md#7-算法-4kd-调度策略)
5. **EMA 力矩滤波** — 5Hz 截止平滑力矩 → [详述](control_report.md#8-算法-5力矩-ema-低通滤波)
6. **力位混合状态机** — 3 状态 + 迟滞 → [详述](control_report.md#9-算法-6力位混合状态机)

## 快速使用

```python
from dm_motor import DmDevice, GripperController

dev = DmDevice()
dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn="YOUR_SN")

with GripperController(dev, can_id=0x01, mst_id=0x11) as gripper:
    gripper.move_to_pct(50)              # 移到 50% 开合
    gripper.close_gripper(force_limit=3) # 力限闭合
    gripper.set_torque(2.0)              # 恒力矩

dev.close()
```

## 参数调节

见 [参数速查表](control_report.md#13-参数速查表)。
