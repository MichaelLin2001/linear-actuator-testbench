# 编译与使用

## 当前能力与限制

本工程直接操作 J3 直线电缸，不执行关节角、连杆运动学或 Jacobian 转换。

目前可运行：串口 Fz 采集、无载清零、CSP 找接触、CST 原始目标加载（阶梯/线性）、降到零后切回 CSP 退离，以及记录和故障停止。

**按 s 会实际加载。** 目标直接写入 0x6071，反馈直接读取 0x6077；去掉旧的 2.5 推力换算。CDM-ANT 手册将它们定义为目标/实际转矩，但这两行的单位栏为空，无法据此确认 A。所有加载参数、界面和 CSV 使用 raw，不能用于计算 N/A 系数。当前峰值 10 raw、保护上限 20 raw，绝不是 10 A / 20 A。

仓库内置裁剪后的单 J3 驱动；三个终端分别运行驱动、controller 和力传进程。controller 连接 motor_test 内置命令通道，不自动启动 motor_test。运行期键文件、日志、命令和 socket 使用 /tmp/j3-testbench，新信号量与旧部署隔离；保留原启动初始化（清零目标和反馈数组），不自动删除 IPC。

## 1. Linux 编译

需要 C++17 编译器、CMake 3.20+、Python 3、PyYAML、Bash、系统 readline 开发包和 Boost 头文件，以及串口访问权限。Python 只有 PyYAML 一个第三方依赖，其余模块使用标准库；直接使用电脑上已有的 Python，无需虚拟环境。以下命令从仓库根目录执行：

```bash
# Debian/Ubuntu 示例；其他系统安装同等依赖
sudo apt-get install build-essential cmake python3 python3-yaml libreadline-dev libboost-dev

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

预期测试：`driver_contracts`（驱动使能和绝对/相对移动，使用离线桩）、`offline_contracts`（共享内存单位、状态机与位置轨迹）、`experiment_contracts`（电流轨迹、测量保护、目标槽位保持和力样本处理）、`yaml_contracts`（配置校验）、`process_contracts`（力共享内存、进程标识、调零版本、串口断连和每轮记录）、`command_session_contracts`（直接启动时的命令响应、重复启动保护和退出清理）。这些测试不启动电机或传感器。

CMake 构建把 `jointctl` 和驱动管理脚本复制到 build/bin；修改 Python 工具后需重新执行 CMake 配置，使复制版本更新。只修改 YAML 不需要配置或编译。

非 Debian/Ubuntu 环境，可在已有 Python 中运行 `python3 -m pip install PyYAML`；若提示系统环境受管理，使用系统包管理器安装，不强制覆盖。已经能运行 `python3 -c "import yaml; print(yaml.__version__)"` 就无需重复安装。控制器仍在 Linux 上运行。

## 2. 内置驱动和硬件

单轴映射固定为 J3（共享内存索引 3）、EtherCAT slave index 0、CANopen node 1，TX/RX/CFG passage 为 1/2/3。其它电机应用、参数和全身映射不参与构建。

- 程序与脚本统一位于 `build/bin`；DriverManager 从 `/proc/self/exe` 定位命令投递脚本；motor_test 自己接收命令、写响应和 PID，不需要 Expect。
- 反馈/目标键文件：`/tmp/j3-testbench/feedback_handle`（ID 2）和 `target_handle`（ID 1）。
- 信号量：`/j3_testbench_feedback_mutex` 和 `/j3_testbench_target_mutex`。
- 共享内存 ABI：30 个 16 字节槽，共 480 字节，只控制索引 3；驱动位置/速度单位 mm/mm/s，上层 m/m/s。
- 默认 EtherCAT 网口 `enp2s0`；仅通过 `JC_ETHERCAT_IFACE` 覆盖。

不要单独启动旧 motor_test，也不要与另一份驱动或目标写入程序并行运行。SOEM 和 cpp-readline 源码与许可证随仓库提供，构建静态库；GNU readline 使用系统动态库。

传感器为 M3924A-1X，单轴 Fz，目录量程 10 kN。量程不等于台架安全上限；具体拉压范围、精度和过载条件以设备校准资料为准。

力传进程独占串口。串口协议为 115200/8N1（波特率可配），发送 `AT+GSD` 开始、`AT+GSD=STOP` 结束；帧头 AA 55，长度和包号大端，偏移 6 的 float32 Fz 小端、单位 N。当前解析器不校验帧尾/校验和。协议不匹配不能使用。

### 直接驱动命令

以下命令由用户在完成离线验证后手动执行；不要与 controller 同时操作驱动。

```bash
cd build/bin
sudo ./motor_test
```

驱动终端输入 `AppStart POS`、`AppM18Enable EN_DRV`、`AppM18Status`，确认状态后再运动。

- `AppWrite 3 <absolute_mm>`：绝对位置目标，单位 mm。
- `AppMoveJ3Relative <delta_mm>`：相对于本次命令执行时的实测位置，单位 mm；例如 `AppMoveJ3Relative 0.1`。
- 两个移动命令共用已使能、CSP 和反馈一致性检查；不再限制单次位移；允许有限数值且目标可表示为驱动的有符号 32 位位置计数。零位移不运动。发送步进仍为 0.01 mm，名义周期 1 ms（约 10 mm/s）。直接驱动不读取 YAML 行程限位，目标是否在机械行程内由操作者确认。
- `AppM18Enable RESET`、`AppM18Enable EN_DRV` 和相对移动不再需要 `CONFIRM`；使能成功后保持 POS 周期发送。使能失败、`DIS_DRV`、`RESET` 和 `AppStart STOP` 停止周期发送。
- 驱动不读取 YAML。退出前由用户执行 `AppM18Enable DIS_DRV` 并确认失能。

## 3. 修改 YAML

直接编辑唯一配置 `joint_controller/profiles/experiment.yaml`。文件已填完整示例参数，能用于校验与离线预览；示例不是实机标定值。运行 p 前核对伸长正负号、绝对行程、力限制、接触阈值和退离距离。

| 配置组 | 参数含义 |
|---|---|
| experiment | `direction`: extend/retract；`mode`: staircase/linear；`position_mode`: relative/absolute，缺省为 relative；`max_current_raw`: 包含保持基线的最大原始目标（整数）；`extend_position_sign`: 伸长时位置增加为 +1、减少为 -1，须按实际驱动映射填写 |
| staircase | `levels`: 加载档数；`transition_s`: 档间斜坡；`settle_s`: 等待时间；`record_s`: 每档采集时间 |
| linear | `rise_s`: 上升时间；`peak_hold_s`: 顶点保持；`fall_s`: 下降时间 |
| preload | `speed_mm_s`: 靠近速度；`contact_force_n`: 清零后力幅值阈值；`contact_confirm_s`: 连续接触确认时间；`max_travel_mm`: 最大寻找位移；`timeout_s`: 寻找超时 |
| unload | `current_slew_raw_s`: 原始目标卸载速率（raw/s）；`retract_distance_mm`、`retract_speed_mm_s`: 退离接触位移和峰值速度 |
| limits | `current_raw`、`force_n`: 上限；`position_min_mm`、`position_max_mm`: 电缸绝对位置边界；`velocity_mm_s`: 实测速度上限；`contact_displacement_mm`: 接触后的位移上限 |
| limits | `force_timeout_ms`: 力年龄上限；`experiment_timeout_s`: 整轮超时，包含找接触和退离；`heartbeat_timeout_s`: 终端心跳超时 |
| sensor | `device`: 串口设备；`baud`: 115200、230400、460800 或 921600，必须与设备一致 |
| output | `directory`: 输出目录，相对于 YAML 所在目录解析 |

`position_mode: relative` 在每次 Preload 开始记录实测位置为固定原点，找接触、保持和退离复用该原点；`absolute` 使用绝对坐标。两模式机械轨迹一致，输出、限位及日志始终为绝对坐标，配置仅在 IDLE 修改。

默认轨迹时间：阶梯 5 档，过渡 1 秒、等待 1 秒、采集 3 秒；线性上升 10 秒、保持 3 秒、下降 10 秒。最大原始目标必须大于 READY 静止期间同方向反馈的平均值（取整），且不超过配置 raw 上限；峰值和上限均须为 1–32767 的整数。READY 静止后至少采集 100 个控制周期再允许加载。保持反馈为零允许从零开始，不能据此证明接触载荷为零。

力传终端按 z 执行软件清零（操作者确认无载、静止），约采集 1 秒、至少 80 个新样本，极差需不超过 100 N。这个旧判据不是高精度校准保证。内置力有效性最长 100 ms，YAML 可设置更短检查时间；更长配置不能延长内置有效性。

## 4. 不连接硬件的检查与预览

预览目录可先用 `mkdir -p /tmp/j3-testbench` 创建。

```bash
./build/bin/jointctl experiment \
  --config joint_controller/profiles/experiment.yaml --check-config

./build/bin/jointctl experiment \
  --config joint_controller/profiles/experiment.yaml \
  --preview /tmp/j3-testbench/actuator-staircase-preview.csv --preview-initial-raw 0
```

将 YAML 的 mode 改为 linear 即可预览线性轨迹。预览调用 C++ 中同一条轨迹，**在打开串口、共享内存或驱动之前返回**；`preview-initial-raw` 只是设想的预载幅值，不是实测结果。输出包含时间、假想电流幅值、阶段和档位，已有同名文件拒绝覆盖。

## 5. 三终端台架操作

均从 `/home/nvidia/lyc/linear-actuator-testbench` 执行。正式实验中不要在另一份 motor_test 手动 write，也不要重复启动驱动。

终端 1：直接启动驱动：

```bash
cd build/bin
sudo ./motor_test
```

该终端保留原来的交互命令，并显示 controller 的命令和响应。motor_test 内置同线程命令通道；controller 负责目标、使能和失能。controller 退出不关闭驱动。确认 controller 已退出且电机已失能后，在终端 1 输入 `quit` 正常关闭；实验中不退出驱动终端。

终端 3：力传采集、共享内存发布和显示：

```bash
sudo ./build/bin/force_sensor --device /dev/ttyUSB0 --baud 115200
```

原始力、零偏、净力以 N 显示，约 10 Hz 刷新；按 z 采集约 1 秒均值作为零偏，等“已调零”。这是软件清零，不改传感器内部参数。q / Ctrl-C 正常停止采集并发布无效状态。程序启动/重启需要重新调零。串口参数在此命令指定，controller 不打开串口。

终端 2：在驱动和力传启动后，启动实验 controller：

```bash
sudo ./build/bin/jointctl experiment --config joint_controller/profiles/experiment.yaml
```

先按现场情况修改 YAML 示例参数。controller 读取电机反馈，写电机目标；只读力共享内存 `/tmp/j3-testbench/force_handle`，信号量 `/j3_testbench_force_mutex`。每次读取完整快照，锁忙时复用尚未过期的缓存，不无限等待。力进程唯一发布者通过文件锁防止重复启动。异常杀死进程恰好占有信号量时不会自动重置信号量，需全部读写进程停机后处理。

| 按键 | 操作 |
|---|---|
| c | IDLE 时重读配置 |
| p | 检查新鲜有效力、清零及限制，先创建本轮文件，再使能并预紧 |
| s | READY 静止后确认驱动 CST 模式，执行实际 raw 阶梯/线性加载 |
| u | 加载时先降 raw 目标至零，确认切回 CSP 后退离；找接触/READY 时直接退离；完成后失能并结束记录 |
| 空格 / Esc / Ctrl-C | 急停请求并请求驱动失能，不自动退回 |
| r | 确认失能后复位 IDLE，允许下一轮预紧 |
| q | 找接触/保持/加载时先卸载，随后确认失能、保存并退出 controller |

操作顺序：力终端 z → 已调零 → controller p → READY → s → 自动加载/卸载/退离（或按 u 提前卸载）→ COMPLETE/STOP → r 或 q。正常预紧按现有力阈值找接触并保持位置，不是恒力预紧。

controller 不刷新力读数，力值只在力终端显示。预紧至卸载结束期间，力失效/超时、重新调零、力进程重启或零偏版本改变都会停止实验并记录原因。力终端退出即停止采集，实验中不可关闭。没有给力进程反向控制通道，因此实验中按 z 会导致中止，不会静默更换零偏。

## 6. 急停与故障

另一个终端也可执行：

```bash
sudo ./build/bin/jointctl estop
sudo ./build/bin/jointctl status
sudo ./build/bin/jointctl disable
```

急停数据报由独立线程接收、控制循环优先锁定故障，不等待普通命令或驱动查询。CST 故障时控制循环尝试提交零目标；停止更新后驱动的 100 ms 目标序列看门狗持续请求零目标和失能。 驱动失能仍使用同目录脚本：若主线程正在执行使能/查询，会在它返回后处理失能，不能承诺电气停止在 1 ms 内完成。读 `driver_disabled_confirmed` 判断确认结果；发送请求不等于已停机。

终端心跳超时触发相同故障；controller 崩溃或停止更新由驱动 CST 软件看门狗处理；驱动进程、EtherCAT 或电源自身故障不在该软件看门狗覆盖范围内。键盘操作不能替代硬件急停。失能或零目标也不证明机械载荷已经释放，需看实测力和台架。

力失效、反馈故障、超力、速度/行程/接触后位移超限、跟踪误差超过原 0.25 mm、实验超时及日志错误都停止实验，不自动重启。CST 加载期间检查目标和 0x6077 原始反馈幅值是否超过 raw 上限。

socket 已存在时拒绝启动第二个控制器。先确认旧程序结束、驱动失能后再清理 `/tmp/j3-testbench/controller.sock` 和 `.urgent`；不要在程序运行时删除。

## 7. 数据与后续实机验收

正式 CSV 在 p 请求通过前置检查后、使能/预紧动作之前创建。待机与调零阶段不记录正式 CSV。每轮文件名为 `experiment_日期时间_标识.csv.run序号.csv`；从预紧准备开始记录，到卸载并处理失能结束，或故障停止处理结束。故障和使能失败也保留数据；失能未确认会显示 STOP NOT CONFIRMED，不能继续下一轮。

每轮保存主 CSV、`.summary.csv` 和 `.config.yaml`，controller 会话另保存 `.daemon.log`。配置快照使用合法 YAML 的 JSON 格式，保存 controller 接受的归一化参数，位置/速度单位 m、m/s；不是现场 YAML 的逐字复制。旧文件拒绝覆盖。

主 CSV 包含 `record_time_s`（本轮首条记录为 0）、绝对位置/目标 m、速度 m/s、effort、原始 Fz N、零偏 N、净力 N、包号、接收时间、样本序号、力进程会话和零偏版本、有效性、新样本标记、阶段、方向和故障状态。`commanded_current_raw` 为取整后的目标，`actual_current_raw` 为 0x6077 原始反馈，`actual_current_valid` 表示反馈快照有效；这些列都不是 A。

共享内存只保存最新力样本，不保证串口逐帧无损保存。力统计仅使用有效且 `force_new_sample=1` 的行；相同样本在多个控制周期出现不属于新采样。力时间为主机接收时间，电机共享内存没有源更新序列/时间，因此不能声称硬件同步。

先验收独立力采集和调零，再验收 controller 预紧/退离及数据保存、传感器断连和重新调零停止。再现场验收低 raw 加载、CSP/CST 交接和卸载；驱动确认模式前不开始轨迹，切换失败则请求失能。物理单位比例仍需厂家说明或标定。

每轮 summary 的时序与读取统计仅汇总实际保存的 CSV 行；丢弃行数另列。力失效、会话或零偏变化记录为 `FORCE_SENSOR_INVALID_OR_ZERO_CHANGED`，可对照 CSV 的会话、版本、有效性和时间戳定位原因。

### 更新后首次运行

机构安全且失能时，退出 controller，再退出并重启 motor_test，最后重启 jointctl；力传进程可保持运行。必须同时更新驱动和 controller；旧驱动缺少 raw 协议标识时 s 会拒绝。

共享内存布局仍为 30×16 字节；J3 反馈 reserved[0]=2 表示 raw 加载协议，reserved[1] 表示驱动看门狗故障；目标 reserved[0:2] 为 16 位循环更新序号。其它目标写入程序不可同时运行。

CSP→CST 以 READY 静止反馈平均值作为初值，验证 0x6071 回读，再验证模式 10；CST→CSP 必须先到零，以当前实测位置设置 0x607A 并确认模式 8。切换期间位置偏移超过 0.02 mm 或状态/回读异常都会失能。手动 AppStart TOR 已拒绝，加载由 controller 的 s 管理。

旧 CSV 的 j3_effort 是旧换算值，不能与新 raw 数据混用。新 summary 标明 CSP_PRELOAD_CST_RAW_LOADING；离线测试通过不代表实际模式切换或载荷标定已经完成。


### 第三个终端的步骤提示

`jointctl experiment` 仅在阶段、档位或命令结果变化时输出中文步骤，不定时刷新屏幕。后台心跳和状态查询照常运行。按 `c` 重读配置，`p` 预紧，待提示机构静止后按 `s` 加载；自动卸载、退离、确认驱动失能并结束记录后才显示本轮完成。按 `r` 准备下一轮，或 `q` 退出。

阶梯加载显示上升/下降、当前档位、稳定等待和采集数据；线性加载显示上升、峰值保持和下降。`u` 或加载中 `q` 触发的卸载显示提前卸载完成。请求已接受不代表动作完成。数据路径只在记录开始和结束时提示。

后台查询期间可暂存一次普通按键；已有操作处理中时，新普通按键会明确提示未执行。空格、Esc、Ctrl-C 急停仍立即发送。故障 11 显示“实验安全保护”，具体原因需结合记录检查；未确认失能时不会显示正常完成。更新后重新启动第三个终端的 jointctl 即可。
