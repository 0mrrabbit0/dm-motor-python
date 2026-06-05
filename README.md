# dm-motor-python

达妙(Damiao)电机的 **Python 控制库** + **CAN/CAN-FD 系统学习笔记** + **完整故障排查文档**。

基于达妙官方的 `DM_DeviceSDK`(`libdm_device.so` / `dm_device.dll`),用 Python ctypes 包装,绕过官方 Python 例程对老 SDK 的依赖,**与最新版适配器固件(1006+)和 DMTool 完全兼容**。

测试硬件:**DM-USB2FDCAN 适配器 + DM-J4310-2EC 电机(固件 V5017_04)**,在 5Mbps CAN-FD 下可达 1kHz 控制频率。

---

## 项目结构

```
├── src/dm_motor/          # 核心库（pip install -e . 后可 import）
│   ├── __init__.py
│   ├── sdk.py                 # 底层 CAN 通信 + 4 种控制模式
│   └── gripper.py             # 夹爪控制器（力位混合）
├── examples/              # 使用示例
│   ├── classic_can_mit.py     # 1Mbps 经典 CAN MIT 模式
│   ├── canfd_vel.py           # 5Mbps CAN-FD VEL 模式
│   ├── canfd_mit.py           # 5Mbps CAN-FD MIT 模式（推荐起点）
│   └── gripper_demo.py        # 夹爪三模式交互 demo
├── tools/                 # 实用工具
│   ├── set_baud.py            # 写电机波特率到 flash
│   ├── read_params.py         # 读电机关键寄存器
│   ├── probe_one.py           # 单组合 CAN-FD 参数探测
│   └── probe_runner.sh        # 批量探测 runner
├── vendor/                # 第三方二进制依赖
│   └── dm_device_sdk/         # 达妙 DM_DeviceSDK (v1.1.0)
│       ├── include/               # C 头文件 (dmcan.h)
│       └── linux/{x86_64,arm64}/  # 平台原生 .so
├── docs/                  # 文档
│   ├── learning_notes.md      # CAN/CAN-FD 系统学习笔记
│   ├── troubleshooting.md     # 实战故障排查文档
│   ├── gripper_design.md      # 夹爪控制设计文档
│   └── CHANGELOG.md           # 版本变更记录
└── pyproject.toml         # 项目元数据
```

---

## 快速开始

### 前置硬件

- 达妙 USB-CANFD 适配器(VID 0x34B7 / PID 0x6877),固件版本 1006(出厂值)
- DM-J4310-2EC 电机(或类似的 DM 系列),固件 V5017_04
- 24V 电源(隔离)
- CAN 接线:H/L 三线,**适配器 GND ↔ 24V 电源负极**(信号共地必须接)
- 终端电阻:适配器内置 120Ω,电机端外接 120Ω → 总线电阻 ~60Ω

### 软件依赖(WSL/Linux)

| 依赖 | 来源 | 安装 |
|---|---|---|
| `libdm_device.so` | 已内置于 `vendor/dm_device_sdk/` | 无需额外操作 |
| `libusb 1.0` | 系统包管理器 | `sudo apt install libusb-1.0-0-dev` |
| Python 3.10+ | system 或 conda | — |

SDK 二进制文件已放在 `vendor/dm_device_sdk/linux/{x86_64,arm64}/`,代码会自动检测平台架构加载对应 `.so`。`libusb` 也会自动从 `LD_LIBRARY_PATH`、conda 环境、系统路径中搜索。

详细环境配置见 [`docs/troubleshooting.md` 第 7-9 条](docs/troubleshooting.md)。

### 安装

```bash
# 在项目根目录，以可编辑模式安装
pip install -e .
```

安装后即可在任意位置 `from dm_motor import DmDevice`。

### 跑通 5M CAN-FD MIT 模式控制

```bash
# 适配器先 attach 到 WSL: usbipd attach --wsl --busid <id>
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib python -u examples/canfd_mit.py
```

电机会以约 2 rad/s 转 3 秒,实时打印位置/速度/扭矩反馈。

### 夹爪力位混合控制

```bash
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib python -u examples/gripper_demo.py
```

交互菜单支持三种模式:
- **纯位置**: 无视外力,直接移动到目标角度
- **纯力矩**: 施加恒定力矩,不管位置
- **力位混合**: 向目标运动,遇到设定力则停住/柔顺跟随

也可以在代码中直接调用:

```python
from dm_motor import DmDevice, GripperController

dev = DmDevice()
dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, canfd=True, brs=True)

with GripperController(dev, can_id=0x01, mst_id=0x11) as gripper:
    gripper.close_gripper(force_limit=3.0)   # 力位混合闭合
    gripper.move_to(1.57)                    # 纯位置到 90°
    gripper.set_torque(2.0)                  # 纯力矩 2 Nm

dev.close()
```

详细设计见 [`docs/gripper_design.md`](docs/gripper_design.md)。

---

## 关键设计取舍

**为什么不用官方 Python 例程?**
官方 [`5.控制例程/电机控制例程/Python例程/u2canfd/`](https://gitee.com/kit-miao/motor-control-routine) 用的是**老版 SDK**,与适配器固件 ≥1004 不兼容(详见 [`docs/troubleshooting.md` 第 2 条](docs/troubleshooting.md))。出厂适配器是 1006,跑官方例程会报 `CMD_SETUP_BUARD ACK FAIL`。要么把适配器降到 1003(失去 DMTool 支持),要么用本仓库的 ctypes 包装(保留 1006 + DMTool)。

**为什么用 ctypes 而不是写 cffi/Cython 绑定?**
ctypes 是 Python 标准库,**零依赖**;DM_DeviceSDK 的导出符号简单(15 个 C 函数),手写包装 ~250 行就够。

**为什么挑 MIT 模式作主要例子?**
MIT 模式是 4 种控制模式中最灵活的(力位混合的扩展),通过参数组合可退化成纯位置/纯速度/纯力矩,**一种命令格式覆盖大多数场景**。VEL 模式虽然简单(只发 4 字节速度)但只能控速度。

---

## 相关链接

- 达妙官方仓库(Gitee): https://gitee.com/dmBots/dmBot
- DM-J4310 中文说明书: 在 dm-tools 仓库 `1.关节电机/DM-J4310-2EC/说明书/`
- DM_DeviceSDK 头文件: `dm-tools/DM_DeviceSDK/C&C++/lib/pub_user.h`
- 达妙论坛: https://bbs.dmbot.cn

---

## License

代码部分:MIT License。文档部分:CC BY 4.0。

本仓库**不包含**达妙官方的二进制 SDK 文件(`libdm_device.so` 等),请从达妙官方 [dm-tools](https://gitee.com/kit-miao/dm-tools) 仓库自行获取。
