# dm-motor-python

达妙(Damiao)电机的 **Python 控制库** + **CAN/CAN-FD 系统学习笔记** + **完整故障排查文档**。

基于达妙官方的 `DM_DeviceSDK`(`libdm_device.so` / `dm_device.dll`),用 Python ctypes 包装,绕过官方 Python 例程对老 SDK 的依赖,**与最新版适配器固件(1006+)和 DMTool 完全兼容**。

测试硬件:**DM-USB2FDCAN 适配器 + DM-J4310-2EC 电机(固件 V5017_04)**,在 5Mbps CAN-FD 下可达 1kHz 控制频率。

---

## 项目结构

```
├── src/dm_motor/          # 核心库（pip install -e . 后可 import）
│   ├── __init__.py
│   └── sdk.py
├── examples/              # 使用示例
│   ├── classic_can_mit.py     # 1Mbps 经典 CAN MIT 模式
│   ├── canfd_vel.py           # 5Mbps CAN-FD VEL 模式
│   └── canfd_mit.py           # 5Mbps CAN-FD MIT 模式（推荐起点）
├── tools/                 # 实用工具
│   ├── set_baud.py            # 写电机波特率到 flash
│   ├── read_params.py         # 读电机关键寄存器
│   ├── probe_one.py           # 单组合 CAN-FD 参数探测
│   └── probe_runner.sh        # 批量探测 runner
├── docs/                  # 文档
│   ├── learning_notes.md      # CAN/CAN-FD 系统学习笔记
│   └── troubleshooting.md     # 实战故障排查文档
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

[`libdm_device.so`](https://gitee.com/kit-miao/dm-tools) 来自达妙官方仓库 `dm-tools/DM_DeviceSDK/`(本仓库不打包,按下面路径配置):

| 依赖 | 来源 | 安装 |
|---|---|---|
| `libdm_device.so` | [dm-tools/DM_DeviceSDK/C&C++/lib/linux/](https://gitee.com/kit-miao/dm-tools) | 直接用,不需安装 |
| `libstdc++ ≥ GLIBCXX_3.4.32` | conda-forge | `bash Miniforge3-Linux-x86_64.sh` |
| `libusb 1.0.27+` | conda-forge | `conda install -y libusb=1.0.27` |
| Python 3.10+ | system 或 conda | — |

详细环境配置见 [`docs/troubleshooting.md` 第 7-9 条](docs/troubleshooting.md)。

### 安装

```bash
# 在项目根目录，以可编辑模式安装
pip install -e .
```

安装后即可在任意位置 `from dm_motor import DmDevice`。

### 改 SDK 路径

打开 [`src/dm_motor/sdk.py`](src/dm_motor/sdk.py) 顶部:

```python
SDK_PATH = "/path/to/dm-tools/DM_DeviceSDK/C&C++/lib/linux/libdm_device.so"
LIBUSB_PATH = "/path/to/miniforge3/lib/libusb-1.0.so.0"
```

按你的路径改。

### 跑通 5M CAN-FD MIT 模式控制

```bash
# 适配器先 attach 到 WSL: usbipd attach --wsl --busid <id>
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib python -u examples/canfd_mit.py
```

电机会以约 2 rad/s 转 3 秒,实时打印位置/速度/扭矩反馈。

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
