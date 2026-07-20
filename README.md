# RS03 电流/直接力矩控制（ROS2）

本包依据 RS03 官方私有 CAN 协议实现两种模式：

- `current`：设置 `run_mode=3`，向 `0x7006 iq_ref` 直接写入单位为 A 的 `float32`。底层电流 PI 在驱动器内部运行。
- `torque`：使用通信类型 1 的前馈力矩字段，位置/速度目标及 `Kp/Kd` 均设为 0。协议范围为 ±60 N·m。

状态帧中的力矩应视为驱动器反馈/估算力矩；手册没有说明 RS03 内置独立力矩传感器，因此这里没有实现“实际力矩传感器 + 外部 PID”的力矩闭环。

## 重要修正

官方示例的 `RobStrite_Motor_Current_control()` 把安培值先编码为 `uint16`，再把整数数值作为 `float` 写入参数。`iq_ref` 参数本身要求 IEEE-754 `float`，所以本包直接打包限幅后的安培值。

## 构建

```bash
cd ~/ros2_ws/src
git clone https://github.com/dk6760098/rs03-current-torque-control.git \
  rs03_current_torque_control
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select rs03_current_torque_control
source install/setup.bash
```

CAN 初始化（按你的适配器名称修改 `can0`）：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 100
sudo ip link set can0 up
```

### WSL2 + 串口式 CAN 适配器

如果 `lsusb` 能看到设备且出现 `/dev/ttyUSB0`，并且适配器固件支持
SLCAN 协议，可运行：

```bash
cd ~/ros2_ws/src/rs03_current_torque_control
bash scripts/setup_socketcan.sh slcan can0 /dev/ttyUSB0
```

脚本会加载 `can`、`can_raw`、`slcan`，以 1 Mbit/s 创建 `can0` 并验证
CAN_RAW 套接字。如果 `slcand` 无法创建接口，通常表示串口转换器不使用
SLCAN 协议；此时需要适配器厂商的 Linux SocketCAN 驱动，不能仅凭存在
`/dev/ttyUSB0` 就直接运行本节点。

原生 SocketCAN 适配器已经产生 `can0` 时使用：

```bash
bash scripts/setup_socketcan.sh native can0
```

## 第一次测试

1. 电机固定可靠、输出轴无人员接触，准备急停/断电。
2. 保持配置中的 `auto_enable: false`，先启动并确认 CAN 接口、ID 无误。
3. 将限幅保持为 `1 A` / `2 N·m` 或更低。
4. 确认后才把 `auto_enable` 改为 `true`。

```bash
ros2 launch rs03_current_torque_control rs03_current_torque.launch.py
```

电流模式命令：

```bash
ros2 topic pub --once /rs03_current_torque/current_command_a std_msgs/msg/Float32 "{data: 0.2}"
```

直接力矩模式命令（先把 YAML 的 `control_mode` 改成 `torque`）：

```bash
ros2 topic pub --once /rs03_current_torque/torque_command_nm std_msgs/msg/Float32 "{data: 0.5}"
```

命令必须持续刷新；超过 `command_timeout_s`（默认 100 ms）后输出自动归零。节点退出时会先发零命令，再发送停止帧。

## 与 PID 的关系

- 电流模式：ROS2 给 `iq_ref`，RS03 内部 PI 产生 PWM。
- 直接力矩模式：ROS2 给力矩前馈，驱动器换算并调用内部电流环。
- 若未来接入独立力矩传感器，才适合在 ROS2 中增加低频外部力矩 PI，其输出应为 `iq_ref`，且频率必须低于内部电流环。
