# DM 电机 Python 控制 — 故障排查文档

记录在 WSL Ubuntu 22.04 上用 Python 通过 USB-CANFD 适配器控制 DM-J4310 电机过程中遇到的所有非平凡问题。

**硬件**:DaMiao USB2FDCAN 模块(VID 0x34B7, PID 0x6877)+ DM-J4310-2EC 电机(固件 V5017_04)
**软件栈**:DM_DeviceSDK (`libdm_device.so`) + Python ctypes + miniforge libstdc++/libusb

每个问题包含 4 项必须要素:**现象 / 根因 / 解决方案 / 验证**。

---

## 1. Gitee 子模块匿名 HTTPS 克隆失败

**现象**
```
git submodule update --init -- "1.关节电机" "8.工具和上位机"
fatal: could not read Username for 'https://gitee.com': No such device or address
```

**根因**
- damiao 仓库的子模块全部托管在 `https://gitee.com/kit-miao/...`
- gitee 不允许匿名 HTTPS 克隆,要求交互式输入用户名/令牌
- Claude Code 环境无 tty,不能交互输入

**解决方案**
全局 git URL 重写,强制 gitee HTTPS → SSH(前提:本机已配置 gitee SSH key 且 `ssh -T git@gitee.com` 通):
```bash
git config --global url."git@gitee.com:".insteadOf "https://gitee.com/"
git submodule update --init --recursive -- "<目录>"
```

**验证**
```bash
ssh -T git@gitee.com  # 应返回 "Hi <用户名>! You've successfully authenticated"
```

---

## 2. Python u2canfd 例程与适配器固件 ≥1004 不兼容

**现象**
跑 [`5.控制例程/电机控制例程/Python例程/u2canfd/damiao.py`](../damiao/5.控制例程/电机控制例程/Python例程/u2canfd/damiao.py):
```
Set the data baud rate: 5000000.0
Sampling point: 87.5%
[Error] ACK ack_code err, ack_code = 0x03
[Error] CMD_SETUP_BUARD ACK FAIL
```

**根因**
- Python 例程内部用的是**旧版 SDK**(自带 `usb_class.so`),协议与适配器固件 1004 起改过
- 详见 [`8.工具和上位机/dm-tools/USB2CANFD/SDK/UPDATE.md`](../damiao/8.工具和上位机/dm-tools/USB2CANFD/SDK/UPDATE.md):
  > 旧版 SDK 不支持模块固件新版本。单路 USB2CANFD:`1004` 版本起请谨慎核对兼容性。
- 出厂适配器固件是 `dm_usb2canfd_app_1006.enc`(>1004)→ 旧 SDK 拒绝设置波特率

**解决方案**
两条路,选一条(都验证可跑):

**路线 X:适配器固件刷 1003,跑官方 Python 例程**(2025-XX 实测)
- 前提:**电机必须真的在 5M FD 模式**(CAN_BR=9),不能只信 DMTool UI 显示,要用串口 banner 或 `read_motor_state.py` 核实
- 适配器在 [`8.工具和上位机/dm-tools/USB2CANFD/固件/出厂固件/历史固件/dm_usb2fdcan_app_1003.enc`](../damiao/8.工具和上位机/dm-tools/USB2CANFD/固件/出厂固件/历史固件/dm_usb2fdcan_app_1003.enc),用升级工具刷
- 副作用:DMTool 和我们的 `dm_python_test/` 都用不了(都依赖新 SDK)
- 早期"1003 跑不通"是错误结论 — 实际是当时 motor 在 1M 没真切到 5M(问题 5、6.5);电机一旦确认在 5M FD,1003 + 老 SDK 直接跑通,2 rad/s 旋转、1kHz 控制环

**路线 Y:适配器保留 1006,改用新 SDK(libdm_device.so)的 Python ctypes 包装**
- 见我们 [dm_python_test/dm_sdk.py](dm_sdk.py),与 DMTool 用同一套 SDK
- 优点:DMTool、我们脚本、官方 C++ 例程都能并存
- 缺点:不是"官方 Python 例程"

**验证**
- 路线 X(1003 + 官方例程):cd 到 `Python例程/u2canfd/`,`python3 damiao.py`,应看到 `CMD_SETUP_BUARD ACK SUCCESS` + `id: 1 mode is 3` + 持续打印 `canid is: 1 pos: ... vel: ~2.0 ...` + 电机以 2 rad/s 转
- 路线 Y(1006 + 新 SDK):用 [dm_sdk.py](dm_sdk.py) 跑能正常通信
- 反向验证(适配器在 1006 跑官方例程):仍然 `CMD_SETUP_BUARD ACK FAIL`,**与 motor 状态无关**(实测电机已经在 5M FD 完美状态时也照样失败)— 证实是固件 ↔ SDK 协议冲突,不是配置问题

**附:WORKFLOW.md 的盲点**
- 官方 [WORKFLOW.md](../damiao/5.控制例程/电机控制例程/Python例程/u2canfd/WORKFLOW.md) 没说明它假设适配器固件 ≤ 1003
- 出厂适配器(1006)用户**严格按 WORKFLOW 一字不差跑也会失败**,且失败原因不在 WORKFLOW 任何一步
- 通用教训:跑这种"官方例程",要先**核实示例发布时间和你硬件出厂时间**,以及示例 SDK 的兼容版本说明

---

## 3. CAN-FD 帧 vs 经典 CAN 帧格式错配 → Bus Off

**现象**
DMTool 在 1Mbps 设置下发使能命令:
```
标准帧  数据帧  FDCAN  0x1  发送失败  8  FF FF FF FF FF FF FF FC
```
适配器红灯快闪(Bus Off),电机红灯常亮(失能态),Python 0 RX。

**根因**
- 电机出厂默认 1Mbps **经典 CAN 2.0B**(per 手册:波特率 ≤1Mbps 自动用 2.0B,>1Mbps 才进 FD 模式)
- 适配器虽然设了 1M/1M,但默认发**FDCAN 帧**(FDF 位为隐性)
- 经典 2.0B 接收器看到 FDF=隐性(它把这位当 r0 保留位,期望显性)→ 协议错,**根本不 ACK**
- 适配器发出去没人 ACK → TEC 累加 → Error Passive → Bus Off

**解决方案**
两选一:
- **A**:DMTool 适配器设置里**取消 CAN-FD / BRS 勾选**,只发经典 CAN 2.0B
- **B**:把电机 CAN_BR 改成 ≥5(>1Mbps)进 FD 模式,适配器侧也用 FD

我们最终走了 B(用户要求 5M)。但 A 是最快验证物理层的办法。

**验证**
帧日志「类型」列从 `FDCAN` 变成 `CAN`,「发送状态」从 `发送失败` 变 `发送成功`,适配器 LED 绿灯常亮。

---

## 4. CAN 总线 Bus Off / Error Passive 状态解读

**现象**
适配器红灯快闪 = Bus Off;绿灯快闪 = Error Passive。

**根因**(CAN 协议规定)
| 状态 | TEC 阈值 | 含义 |
|---|---|---|
| Active(正常) | TEC<128 | 正常,绿灯常亮 |
| **Error Passive** | TEC≥128 | 错误太多降级,**绿灯快闪**,还能收发 |
| **Bus Off** | TEC≥256 | 完全脱离总线,**红灯快闪**,需重启 |

发送出错(没人 ACK、CRC 错、波形错乱) → TEC +8;成功 → TEC -1。

**解决方案**
根因是物理层/协议层错配,要从根本上解决:
1. 波特率不匹配 → 对齐两端 baud
2. 没接终端电阻 → 两端各 120Ω(总线 H-L 电阻应为 ~60Ω)
3. CAN-FD vs 经典 CAN 错配 → 见问题 3
4. 总线只有自己一个能动的节点 → 检查另一端供电

恢复 Bus Off 的临时手段:**断电重启适配器**(不解决根因还会再进 Bus Off)。

**验证**
- 万用表测 CAN H-L 间电阻应为 ~60Ω(两端有终端) 或 ~120Ω(只一端有)
- 适配器 LED 应稳定绿色

---

## 5. DMTool 下拉改电机参数不持久化

**现象**
- 在 DMTool「电机调试」页改控制模式或波特率下拉、保存
- 断电重启电机后,从串口 banner 看 `CAN Baud: 1.00Mbps`,`Control Mode: 1:MIT Mode`
- **没生效**

但用「参数列表」改 MST_ID 能持久化。

**根因**
- DMTool 下拉框只**改适配器自己的 RAM 状态**,不向电机发任何写参数命令
- 写电机参数必须通过参数列表的「写参数」按钮(底层发 0x55 写命令到 ID 0x7FF,目标 RID)
- 写到 RAM 后必须再发 0xAA save 命令才能持久化到 flash

**解决方案**
改电机参数(CAN_BR、CTRL_MODE、MST_ID 等)有两条路:
1. **DMTool 参数列表**:找到对应 RID 行,改值,点「写参数」,**再点「保存到 flash」**
2. **代码直接发**:见 [set_baud_5m.py](set_baud_5m.py),发写命令 `[idl, idh, 0x55, RID, v0, v1, v2, v3]` 到 ID 0x7FF,再发 save 命令 `[idl, idh, 0xAA, 0x01, 0, 0, 0, 0]`

**验证**
通过串口 banner 看 boot 时打印的实际 flash 值,或者用 [read_motor_state.py](read_motor_state.py) 用读参数命令(0x33)读寄存器。

---

## 6. 关于"电机最高只支持 1Mbps"的错误推论

**现象**
DMTool 改 5Mbps 后 banner 显示还是 1.00Mbps,初步推断"固件不支持 >1M"。

**根因**(实际真相)
- 是问题 5 的副作用:DMTool 下拉根本没写到电机
- 电机固件 V5017_04 实际**支持 CAN_BR=9 (5Mbps)**
- 手册寄存器表写「范围 [0,4]」与同手册的波特率表(0~9 到 5M)矛盾,实测以波特率表为准

**解决方案**
用 [set_baud_5m.py](set_baud_5m.py) 在 1Mbps 经典 CAN 下写入 RID=35=9 + save,电机自动重启到 5M FD 模式:
```bash
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib /home/ubuntu/miniforge3/bin/python -u set_baud_5m.py
```

**验证**
脚本输出 `write ACK with stored val = 9`(电机回 ACK 确认值),断电重启后串口 banner 显示 `CAN Baud: 5.00Mbps`。

**教训**:**别根据"DMTool UI 没生效"就下结论说固件不支持**,先用底层协议直接验证。

---

## 6.5. 跑官方 WORKFLOW 时没核实"上位机配置已生效",凭口头确认就启动 Python

**现象**
WORKFLOW.md 第一步:"首先用最新上位机给电机设置5M波特率"。
我当时听用户说"设置好了"就直接 `python3 damiao.py`,然后撞到 CMD_SETUP_BUARD ACK FAIL。

**根因**
两个问题叠加,我没分清:
1. **DMTool 下拉 UI 实际没把 5M 写进电机**(就是问题 5);电机当时其实在出厂 1M
2. 同时**适配器 1006 + 老 SDK 也确实有协议冲突**(就是问题 2)

我把两个并发问题误归为一个"老 SDK 跟新固件不兼容",没单独诊断,直接走错路(刷 1003 → DMTool 也死)。

**解决方案**
跑任何"按手册一步步配"的流程之前,**对每一步都要有可机器读取的验证手段**,不能只信用户/UI 反馈:
- 电机参数 → 跑 [read_motor_state.py](read_motor_state.py) 通过 CAN 读 RID 实际值
- 串口 banner 再核对一遍(banner 是 boot 时打印,代表 flash 真实值)
- 适配器固件版本 → DMTool 顶部状态栏 / 升级工具里看
- SDK 版本 → 跑脚本时看 `Damiao Device SDK Version: x.x.x.x`

**验证**
现在(motor 已确认 5M FD)再跑官方 damiao.py 仍然 `CMD_SETUP_BUARD ACK FAIL` — **证实这个错误跟 motor 无关**,纯固件冲突。如果当时先核实电机状态,会更快锁定根因。

**教训**:
- 用户口头"设置好了"≠ 真生效。**对自己工作的输入永远要再核验一次**
- WORKFLOW 类文档常常假设默认硬件状态(出厂),不一定与你当前状态一致;每一步配完都用底层手段核对

---

## 7. libdm_device.so 缺少 GLIBCXX_3.4.31 / 3.4.32

**现象**
```python
OSError: /lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.32' not found
```

**根因**
- `libdm_device.so` 用 GCC 13+ 编译,需要 libstdc++ ≥ GLIBCXX_3.4.32
- Ubuntu 22.04 默认只到 GLIBCXX_3.4.30(libstdc++ 12.3)

**解决方案**
不动系统 libstdc++(避免连锁升级 glibc),装 miniforge 自带的新版:
```bash
curl -L -o /tmp/miniforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash /tmp/miniforge.sh -b -p /home/ubuntu/miniforge3
# miniforge 自带 libstdc++ 到 GLIBCXX_3.4.34
```
跑脚本时:
```bash
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib /home/ubuntu/miniforge3/bin/python ...
```

**验证**
```bash
strings /home/ubuntu/miniforge3/lib/libstdc++.so.6 | grep "^GLIBCXX_" | sort -V | tail -3
# 应有 GLIBCXX_3.4.32 / 3.4.33 / 3.4.34
```

**避坑**:别用 `/snap/.../libstdc++.so.6.0.33`,它依赖 GLIBC_2.38(系统是 2.35),会拉一连串 glibc 升级问题。

---

## 8. libdm_device.so 不显式链接 libusb

**现象**
```python
OSError: libdm_device.so: undefined symbol: libusb_open
```

**根因**
- SDK 用 libusb 但**没在自己的 `.so` 里链接 libusb-1.0.so**
- 期望宿主进程已经加载了 libusb,所以 `libusb_*` 符号在运行时由进程的全局符号表解析
- C++ 程序通常 `-lusb-1.0` 链接,自然在进程里;Python ctypes 默认不会预加载

**解决方案**
在 Python 加载 libdm_device 之前,**用 RTLD_GLOBAL 模式预加载 libusb**:
```python
import ctypes
ctypes.CDLL("/path/to/libusb-1.0.so.0", mode=ctypes.RTLD_GLOBAL)
sdk = ctypes.CDLL("/path/to/libdm_device.so")
```
见 [dm_sdk.py](dm_sdk.py) 顶部的 LIBUSB_PATH 处理。

**验证**
`CDLL("libdm_device.so")` 不再抛 undefined symbol,能继续调用 SDK 函数。

---

## 9. libdm_device.so 需要 libusb_init_context (libusb 1.0.27+)

**现象**
编译 C++ 例程时:
```
undefined reference to `libusb_init_context'
```
Python 通过 `RTLD_GLOBAL` 预加载系统 libusb 1.0.25 跑能跑,但 SDK 内部行为可能受限。

**根因**
- `libusb_init_context` 是 libusb 1.0.27 新增的 API
- Ubuntu 22.04 自带 libusb 1.0.25 没有

**解决方案**
用 conda 装 libusb 1.0.27:
```bash
/home/ubuntu/miniforge3/bin/conda install -y libusb=1.0.27
```
然后 dm_sdk.py 里的 `LIBUSB_PATH` 指向 `/home/ubuntu/miniforge3/lib/libusb-1.0.so.0`。

**验证**
```bash
ls /home/ubuntu/miniforge3/lib/libusb-1.0.so.0  # 存在
nm -D /home/ubuntu/miniforge3/lib/libusb-1.0.so.0 | grep libusb_init_context
```

---

## 10. 5M FD 模式下 ACK 错误初次诊断 → C++ 也复现 → 后来突然好了

**现象(初次)**
- 已确认电机 CAN_BR=9 (5M FD),DMTool 在 5M FD 下能读电机参数(双向通信正常)
- 我们的 Python 脚本设 5M FD 后,每发一帧触发一个 err_callback,id=0x1FFFFFF, payload byte[3]=0x10(ACK 错误)
- 0 RX 帧,适配器红灯快闪
- **同样的 SDK 调用写成纯 C++ 也复现完全一样的错误**(byte 级一致)→ 排除 ctypes wrapper 问题

**根因**(未完全定论)
- 当时怀疑 Linux 版 `libdm_device.so` 在 5M FD 下 bit timing 计算有 bug
- 后来用户在 DMTool 里又操作了一些(可能让电机的 FD 状态机重新初始化)
- 之后再跑同样的 Python 脚本,**5M FD 突然完全正常**(30 RX 帧,err=1 使能态)

**解决方案**(临时)
若遇到此现象:
1. 在 DMTool 里**主动**做一次 5M FD 通信(读参数 / 使能),让电机 FD 状态机进入"被驱动过"的状态
2. 然后切回 WSL 跑 Python 脚本
3. 通常能恢复

**验证**
跑 [probe_one.py](probe_one.py) `1 0.75 0.75`,看到 `RESULT brs=1 ... rx_count=30+` 而非 0。

**未解之谜**:Linux SDK 5M FD 初始状态需要被某种方式"激活",根因不明。如反复出现,联系达妙官方报 bug。

---

## 11. VEL 模式命令不动电机 — 因为电机其实在 MIT 模式

**现象**
- 电机使能成功(`err=1`),反馈帧持续返回
- 我们调 `control_vel(0x01, 2.0)`(发 4 字节 float 到 ID 0x201)200 次
- 反馈位置 q ≈ 0.094 rad **一直不变**,速度 dq ≈ 0
- 电机一动不动

**根因**
- 串口 boot banner 显示 `Control Mode: 3:speed Mode <----`,我们以为是 VEL 模式
- 但用 [read_motor_state.py](read_motor_state.py) 通过 CAN 读 RID=10 实际值是 **`CTRL_MODE = 1`**(MIT 模式)
- DMTool 下拉改 CTRL_MODE 不持久化(同问题 5),banner 里那个 "<----" 可能只是 RAM 临时态或 UI 误显
- 电机在 MIT 模式只接收 ID 0x001 的 8 字节 MIT 编码命令,完全忽略 0x201 的 VEL 命令

**解决方案**
两选一:
- **A**:改用 MIT 命令控制(更通用,不需要改电机),见 [test_motor_5m_mit.py](test_motor_5m_mit.py):
  ```python
  dev.control_mit(0x01, kp=0, kd=2.0, q=0, dq=2.0, tau=0)  # 跟随 2 rad/s
  ```
- **B**:写 RID=10=3 + save 把电机切到 VEL 模式,再用 control_vel

实测 A 跑通,电机 3 秒走了 5.2 rad(约 0.83 圈,平均 1.7 rad/s)。

**验证**
```bash
LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib /home/ubuntu/miniforge3/bin/python -u read_motor_state.py
# 看 CTRL_MODE 实际值;跟串口 banner 对比,以读寄存器值为准
```

**通用教训**:**串口 banner 不是 ground truth,要用 CAN 读寄存器实际值**。

---

## 关键 MIT 控制法速查

```python
dev.control_mit(can_id, kp, kd, q_des, dq_des, tau)
```

| 想做的 | kp | kd | q_des | dq_des | tau |
|---|---|---|---|---|---|
| 持位到 q | 20 | 2 | q | 0 | 0 |
| 慢速旋转 ω | 0 | 2 | 0 | ω | 0 |
| 阻尼自由(可手扳) | 0 | 1 | 0 | 0 | 0 |
| 力矩控制 | 0 | 0 | 0 | 0 | N·m |
| 软停 | 0 | 2 | — | 0 | 0 |

---

## 运行环境检查清单

正常工作时应该满足:

```bash
# 1. 适配器接到 WSL
lsusb | grep 34b7
# Bus 001 Device NNN: ID 34b7:6877 DaMiao-Tech DM-USB2FDCAN

# 2. 设备节点权限
ls -l /dev/bus/usb/001/NNN
# 应为 crw-rw-rw-(0666)。如非,设 udev 规则:
# echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb.rules

# 3. miniforge 在位
ls /home/ubuntu/miniforge3/bin/python
strings /home/ubuntu/miniforge3/lib/libstdc++.so.6 | grep "^GLIBCXX_3.4.32"
ls /home/ubuntu/miniforge3/lib/libusb-1.0.so.0

# 4. SDK 文件在位
ls "/home/ubuntu/damiao/8.工具和上位机/dm-tools/DM_DeviceSDK/C&C++/lib/linux/libdm_device.so"

# 5. 串口 banner 应显示
# CAN ID:     0x001
# MASTER ID:  0x011
# CAN Baud: 5.00Mbps
```
