# linear-actuator-testbench

基于既有 jc_driver 的单轴直线电缸实验台：受保护的 CSP 预加载、串口轴向测力、YAML 配置与键盘操作。

电流阶梯/线性轨迹可离线预览；因本包缺少实际电流/CST 接口定义，硬件电流输出保持锁定，不将 effort 假定为安培。无运动学或 Jacobian 转换。

构建、参数、操作与停止限制见 [USAGE.md](USAGE.md)。只使用一套源码，不包含底层驱动、完整上位机软件或原始实验数据。
