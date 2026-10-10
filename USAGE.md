# 编译与使用

## 当前能力与限制

本工程直接操作 J3 直线电缸，不执行关节角、连杆运动学或 Jacobian 转换。

目前可运行：YAML 校验、两种电流轨迹的离线预览、串口 Fz 采集、无载清零、受保护的 CSP 找接触和退离接触，以及终端操作、心跳和故障停止。

**实际电流输出锁定。** 包内没有底层驱动的电流/CST 单位、实际电流反馈及模式切换接口定义，程序不会把 `target_effort` 猜成安培。按 `s` 返回明确拒绝，不会发送电流或切换到 CST。阶梯/线性目前通过离线预览验证，不能称为已完成实机电流加载。实际电流字段为空、有效性为 false，不能据此计算 N/A 标定系数。

继续使用原来已部署的 jc_driver，不更换驱动。不创建、清空或删除其共享内存。

## 1. Linux 编译

需要 C++17 编译器、CMake 3.16+、Python 3、PyYAML、Expect、Bash，以及串口访问权限。Python 只有 PyYAML 一个第三方依赖，其余模块使用标准库；直接使用电脑上已有的 Python，无需虚拟环境。以下命令从仓库根目录执行：

```bash
# Debian/Ubuntu 示例；其他系统安装同等依赖
sudo apt-get install build-essential cmake python3 python3-yaml expect

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
chmod +x build/jointctl jc_driver_integration/tools/single_j3/*
```

预期测试：`offline_contracts`（共享内存单位、状态机与位置轨迹）、`experiment_contracts`（电流轨迹、测量保护、目标槽位保持和力样本处理）、`yaml_contracts`（配置校验）。这些测试不启动电机或传感器。

CMake 构建把 `jointctl` 复制到 build；修改 Python 工具后需重新执行 CMake 配置，使复制版本更新。只修改 YAML 不需要配置或编译。

非 Debian/Ubuntu 环境，可在已有 Python 中运行 `python3 -m pip install PyYAML`；若提示系统环境受管理，使用系统包管理器安装，不强制覆盖。已经能运行 `python3 -c "import yaml; print(yaml.__version__)"` 就无需重复安装。控制器仍在 Linux 上运行。

## 2. 原驱动和硬件

以下原部署路径按要求保留：

- 驱动启动/命令脚本：`/home/nvidia/Documents/jc_driver/tools/single_j3/`。
- 驱动程序：`/home/nvidia/Documents/jc_driver/artifacts/single_j3_stage1_final/jc01/bin/motor_test`。
- 反馈键文件：`/home/jc_robot/feedback_handle`，项目 ID `0x02`。
- 目标键文件：`/home/jc_robot/target_handle`，项目 ID `0x01`。
- 信号量：`sem_feedback_handle_mutex`、`sem_target_handle_mutex`。
- 共享内存：30 个 16 字节槽，总计 480 字节，只写索引 3。位置/速度的驱动单位是 mm/mm/s，上层内部是 m/m/s。

驱动管理实际调用原部署目录中的脚本；仓库内 `jc_driver_integration` 是配套脚本副本。原脚本不存在时，先将这三个脚本安装到上述驱动目录；不要同时启动另一份 motor_test 或其他目标写入程序。

传感器为 M3924A-1X，单轴 Fz，目录量程 10 kN。量程不等于台架安全上限；具体拉压范围、精度和过载条件以设备校准资料为准。

串口协议为 115200/8N1（波特率可配），发送 `AT+GSD` 开始、`AT+GSD=STOP` 结束；帧头 AA 55，长度和包号大端，偏移 6 的 float32 Fz 小端、单位 N。当前解析器不校验帧尾/校验和。协议不匹配不能使用。

## 3. 修改 YAML

```bash
cp joint_controller/profiles/experiment.yaml \
   joint_controller/profiles/experiment.local.yaml
```

编辑私人配置，把所有 `null` 填成经过确认的值。模板故意没有硬件安全默认值；测试代码中的合成参数不能复制成实机参数。`*.local.yaml` 不入库。

| 配置组 | 参数含义 |
|---|---|
| experiment | `direction`: extend/retract；`mode`: staircase/linear；`max_current_a`: 包含预载的最大电流；`extend_position_sign`: 伸长时位置增加为 +1、减少为 -1，须按实际驱动映射填写 |
| staircase | `levels`: 加载档数；`transition_s`: 档间斜坡；`settle_s`: 等待时间；`record_s`: 每档采集时间 |
| linear | `rise_s`: 上升时间；`peak_hold_s`: 顶点保持；`fall_s`: 下降时间 |
| preload | `speed_mm_s`: 靠近速度；`contact_force_n`: 清零后力幅值阈值；`contact_confirm_s`: 连续接触确认时间；`max_travel_mm`: 最大寻找位移；`timeout_s`: 寻找超时 |
| unload | `current_slew_a_s`: 将来电流卸载速率；`retract_distance_mm`、`retract_speed_mm_s`: 退离接触位移和峰值速度 |
| limits | `current_a`、`force_n`: 上限；`position_min_mm`、`position_max_mm`: 电缸绝对位置边界；`velocity_mm_s`: 实测速度上限；`contact_displacement_mm`: 接触后的位移上限 |
| limits | `force_timeout_ms`: 力年龄上限；`experiment_timeout_s`: 整轮超时，包含找接触和退离；`heartbeat_timeout_s`: 终端心跳超时 |
| sensor | `device`: 串口设备；`baud`: 115200、230400、460800 或 921600，必须与设备一致 |
| output | `directory`: 输出目录，相对于 YAML 所在目录解析 |

默认轨迹时间：阶梯 5 档，过渡 1 秒、等待 1 秒、采集 3 秒；线性上升 10 秒、保持 3 秒、下降 10 秒。最大电流必须大于预载电流，且不超过配置电流上限。没有实际电流接口时，上述参数只用于离线轨迹。

现有软件清零约采集 1 秒、至少 80 个新样本，极差需不超过 100 N。这个旧判据不是高精度校准保证。内置力有效性最长 100 ms，YAML 可设置更短检查时间；更长配置不能延长内置有效性。

## 4. 不连接硬件的检查与预览

```bash
./build/jointctl experiment \
  --config joint_controller/profiles/experiment.local.yaml --check-config

./build/jointctl experiment \
  --config joint_controller/profiles/experiment.local.yaml \
  --preview /tmp/actuator-staircase-preview.csv --preview-initial-a 0
```

将 YAML 的 mode 改为 linear 即可预览线性轨迹。预览调用 C++ 中同一条轨迹，**在打开串口、共享内存或驱动之前返回**；`preview-initial-a` 只是设想的预载幅值，不是实测结果。输出包含时间、假想电流幅值、阶段和档位，已有同名文件拒绝覆盖。

## 5. 台架操作

先确认夹具受力路径、机械行程及硬件急停。左侧钢板对应伸长加载，右侧对应收缩加载；换钢板前解除载荷并确认失能。首次只做最低载荷测试。

```bash
./build/jointctl experiment \
  --config joint_controller/profiles/experiment.local.yaml
```

入口启动控制器、原驱动及采集线程，但不自动使能。终端约 10 Hz 刷新状态；执行使能/失能等长命令时保留上一条状态，键盘急停和心跳仍通过独立通道工作。

| 按键 | 操作与前置条件 |
|---|---|
| c | IDLE 时重新读取 YAML；串口或输出目录改变需要退出后重启 |
| z | 钢板未受力且电缸静止时清零；等 `zeroed=true` 后继续，不能把预载清掉 |
| p | 校验限制、执行原驱动使能流程，开始低速找接触；连续达到阈值后进入 READY 并保持当前位置 |
| s | 电流接口尚未定义，当前明确拒绝，不发送电流 |
| u | 取消寻找/保持，按 CSP 位置轨迹反向退让；力回到接触阈值以下才标为 COMPLETE |
| 空格 / Esc / Ctrl-C | 急停请求；锁定 FAULT、停止继续写目标，随后请求原驱动失能，不自动退回 |
| r | 完成卸载后停止并确认失能，再复位到 IDLE；故障时必须先确认驱动已失能，不自动重新使能 |
| q | READY/PRELOAD 先卸载，完成后失能并退出；停止未确认时不会显示正常退出 |

本版推荐顺序：`z → 等清零完成 → p → 检查 READY 和受力 → u → COMPLETE → r`。随后 `c` 修改参数、再做一轮，或 `q` 退出。不能把 READY 当作已经进入电流模式。

## 6. 急停与故障

另一个终端也可执行：

```bash
./build/jointctl estop
./build/jointctl status
./build/jointctl disable
```

急停数据报由独立线程接收、控制循环优先锁定故障，不等待普通命令或驱动查询。**本版没有电流输出，所以没有已接通的零电流/CST停止路径。** 驱动失能仍使用原脚本：若主线程正在执行使能/查询，会在它返回后处理失能，不能承诺电气停止在 1 ms 内完成。读 `driver_disabled_confirmed` 判断确认结果；发送请求不等于已停机。

终端心跳超时触发相同故障；controller 本身崩溃需要底层驱动看门狗覆盖，当前包不能确认该能力。键盘操作不能替代硬件急停。失能或零目标也不证明机械载荷已经释放，需看实测力和台架。

力失效、反馈故障、超力、速度/行程/接触后位移超限、跟踪误差超过原 0.25 mm、实验超时及日志错误都停止实验，不自动重启。电流超限检查需实际电流接口接通后才能实现，当前没有实机电流。

socket 已存在时拒绝启动第二个控制器。先确认旧程序结束、驱动失能后再清理 `/tmp/joint_controller.sock` 和 `.urgent`；不要在程序运行时删除。

## 7. 数据与后续实机验收

输出目录含唯一命名 CSV、`.summary.csv`、`.daemon.log` 和每次预加载保存的 `.config.*.yaml`。CSV/配置使用独占创建，不覆盖旧实验。

保留位置、速度、effort、原始 Fz、零偏、清零后力、包号、时间、有效性、新样本标记、阶段和方向。实际电流为空，`actual_current_valid=false`；`commanded_current_a=0` 表示本版未发送电流。

控制周期名义 1 kHz；实际力收帧率看 `receive_hz`。力统计使用有效且 `force_new_sample=1` 的样本。力时间是主机接收时间，电机共享内存没有源更新序列或源时间，不能声称两路硬件同步采样。

在部署 Linux 上重新执行 CMake/CTest，最低载荷验收找接触、退离、键盘急停、传感器断连、终端断连和驱动失能确认。电流接口接通后，还必须验收 CSP/电流模式切换、实际电流、斜坡卸载及驱动看门狗，再开始正式标定。
