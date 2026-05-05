# dm-motor-python

达妙(Damiao)电机的 **Python 控制库** + **CAN/CAN-FD 系统学习笔记** + **完整故障排查文档**。

基于达妙官方的 `DM_DeviceSDK`(`libdm_device.so` / `dm_device.dll`),用 Python ctypes 包装,绕过官方 Python 例程对老 SDK 的依赖,**与最新版适配器固件(1006+)和 DMTool 完全兼容**。

测试硬件:**DM-USB2FDCAN 适配器 + DM-J4310-2EC 电机(固件 V5017_04)**,在 5Mbps CAN-FD 下可达 1kHz 控制频率。

---

## 文件清单

### 代码

| 文件 | 用途 |
|---|---|
| [`dm_sdk.py`](dm_sdk.py) | Python ctypes 包装的 DM_DeviceSDK,核心库 — 经典 CAN + CAN-FD,enable/disable、MIT/VEL 控制、参数读写、save 持久化 |
| [`test_motor.py`](test_motor.py) | 1Mbps 经典 CAN 控制示例(电机出厂默认状态) |
| [`test_motor_5m.py`](test_motor_5m.py) | 5Mbps CAN-FD VEL 模式控制示例(电机需先切到 5M FD VEL 模式) |
| [`test_motor_5m_mit.py`](test_motor_5m_mit.py) | 5Mbps CAN-FD MIT 模式控制示例 — **推荐起点**,空载 2 rad/s 旋转 |
| [`set_baud_5m.py`](set_baud_5m.py) | 通过 CAN 直接写电机 `CAN_BR=9` (5M) 寄存器 + 持久化到 flash,绕过 DMTool 下拉的 bug |
| [`read_motor_state.py`](read_motor_state.py) | 用 CAN 读电机关键寄存器(MST_ID / ESC_ID / CTRL_MODE / CAN_BR 等),核实电机真实状态 |
| [`probe_one.py`](probe_one.py) | 单组合 CAN-FD 参数探测(BRS / 采样点等),用于排查 ACK 错 |
| [`probe_runner.sh`](probe_runner.sh) | 把多个 probe 组合用独立子进程跑,避免 SDK 多次开关污染 |

### 文档

| 文件 | 用途 |
|---|---|
| [`LEARNING_NOTES.md`](LEARNING_NOTES.md) | **CAN/CAN-FD 系统学习笔记** — 从传输线物理到 DM 电机协议层,9 大部分,4 阶段学习路径 |
| [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) | 实战故障排查文档,12+ 条记录,每条含**现象 / 根因 / 解决方案 / 验证** 四要素 |

---

## 快速开始

### 前置硬件

- 达妙 USB-CANFD 适配器(VID 0x34B7 / PID 0x6877),固件版本 1006(出厂值)
- DM-J4310-2EC 电机(或类似的 DM 系列),固件 V5017_04
- 24V 电源(隔离)
- CAN 接线:H/L 三线,**适配器 GND ↔ 24V 电源负极**(信号共地必须接)
- 终端电阻:适配器内置 120Ω,电机端外接 120Ω → 总线电阻 ~60Ω

### 软件依赖(WSL/Linux)

[`libdm_device.so`](https://gitee.com/kit-miao/dm-tools) 来自达妙官方仓库 `dm-tools/DM_DeviceSDK/`(本仓库不打包,按下面路径配置):

| 依赖 | 来源 | 安装 |
|---|---|---|
| `libdm_device.so` | [dm-tools/DM_DeviceSDK/C&C++/lib/linux/](https://gitee.com/kit-miao/dm-tools) | 直接用,不需安装 |
| `libstdc++ ≥ GLIBCXX_3.4.32` | conda-forge | `bash Miniforge3-Linux-x86_64.sh` |
| `libusb 1.0.27+` | conda-forge | `conda install -y libusb=1.0.27` |
| Python 3.10+ | system 或 conda | — |

详细环境配置见 [`TROUBLESHOOTING.md` 第 7-9 条](TROUBLESHOOTING.md)。

### 改 SDK 路径

打开 [`dm_sdk.py`](dm_sdk.py) 顶部:

```python
SDK_PATH = "/path/to/dm-tools/DM_DeviceSDK/C&C++/lib/linux/libdm_device.so"
LIBUSB_PATH = "/path/to/miniforge3/lib/libusb-1.0.so.0"
```

按你的路径改。

### 跑通 5M CAN-FD MIT 模式控制

```bash
# 适配器先 attach 到 WSL: usbipd attach --wsl --busid <id>
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib /home/ubuntu/miniforge3/bin/python -u test_motor_5m_mit.py
```

电机会以约 2 rad/s 转 3 秒,实时打印位置/速度/扭矩反馈。

---

## 关键设计取舍

**为什么不用官方 Python 例程?**
官方 [`5.控制例程/电机控制例程/Python例程/u2canfd/`](https://gitee.com/kit-miao/motor-control-routine) 用的是**老版 SDK**,与适配器固件 ≥1004 不兼容(详见 [`TROUBLESHOOTING.md` 第 2 条](TROUBLESHOOTING.md))。出厂适配器是 1006,跑官方例程会报 `CMD_SETUP_BUARD ACK FAIL`。要么把适配器降到 1003(失去 DMTool 支持),要么用本仓库的 ctypes 包装(保留 1006 + DMTool)。

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

---

🤖 此仓库由 [Claude Code](https://claude.com/claude-code) 协助创建
