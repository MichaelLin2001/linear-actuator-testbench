# linear-actuator-testbench

基于既有 jc_driver 的单轴直线电缸实验台：CSP 预加载、CST 原始目标阶梯/线性加载、串口轴向测力、YAML 配置与键盘操作。

按 s 会实际加载；0x6071 目标和 0x6077 反馈使用 raw，尚未确认到 A 的比例。自动卸载至零后切回 CSP 退离。无运动学或 Jacobian 转换。

构建、参数、操作与停止限制见 [USAGE.md](USAGE.md)。驱动和 controller 必须同时更新，失能后重启生效；统一构建至 build/bin。
