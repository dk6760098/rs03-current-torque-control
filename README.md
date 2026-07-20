# RS03 电流/直接力矩控制（ROS2）

本包依据 RS03 官方私有 CAN 协议实现四种模式：

- `current`：设置 `run_mode=3`，向 `0x7006 iq_ref` 直接写入单位为 A 的 `float32`。底层电流 PI 在驱动器内部运行。
- `torque`：使用通信类型 1 的前馈力矩字段，位置/速度目标及 `Kp/Kd` 均设为 0。协议范围为 ±60 N·m。
- `velocity`：设置 `run_mode=2`，写入速度目标，并配置电流和加速度限制。
- `position_pp`：设置 `run_mode=1`，使用驱动器内部 PP 轨迹，以节点启动位置为基准发送相对位置。

通信后端支持：

- `transport: serial`：灵足 CH340 串口转 CAN，直接使用官方 `AT` 二进制帧，默认 `/dev/ttyUSB0`、921600 baud。
- `transport: socketcan`：Canable/candleLight 等原生 SocketCAN 设备。

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

### 灵足 CH340 串口转 CAN（当前默认）

配置已经默认选择 `/dev/ttyUSB0`。该模式不使用 `slcand`，启动前必须停止
占用串口的旧进程：

```bash
sudo pkill -x slcand 2>/dev/null || true
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs03_current_torque_control rs03_current_torque.launch.py
```

节点启动时只发送安全停止帧并等待反馈。看到
`RS03 feedback received; transport is online` 才表示串口和 CAN 链路真正连通。
没有有效反馈时，即使 `auto_enable=true`，节点也会拒绝使能。

第一次低电流测试时，显式传入使能和限幅：

```bash
ros2 launch rs03_current_torque_control rs03_current_torque.launch.py \
  auto_enable:=true control_mode:=current max_current_a:=0.5
```

在另一个已经 source 工作空间的终端中，以 20 Hz 发送 `0.1 A`：

```bash
ros2 topic pub -r 20 \
  /rs03_current_torque/current_command_a std_msgs/msg/Float32 "{data: 0.1}"
```

停止发布超过 100 ms 后，看门狗会把指令归零、发送停止帧并锁定禁止输出；
必须重启节点才能再次使能。完成测试后仍要停止控制节点，不要依赖软件停机
代替硬件急停。

台架配置还会限制命令变化率，并在反馈速度超过 `2 rad/s` 或温度超过
`60 C` 时发送零命令和停止帧。速度必须连续 5 个反馈样本超限才会触发，
避免编码器速度的单点噪声误停。纯力矩模式在克服静摩擦后会继续加速，速度
保护只是最后一道软件保护，不会把力矩模式变成速度控制。

可观察的反馈话题包括：

```text
/rs03_current_torque/position_rad
/rs03_current_torque/velocity_rad_s
/rs03_current_torque/estimated_torque_nm
/rs03_current_torque/temperature_c
```

## 速度模式

第一次测试建议限制在 `0.2 rad/s`，约为输出轴 `1.9 rpm`：

```bash
ros2 launch rs03_current_torque_control rs03_current_torque.launch.py \
  auto_enable:=true control_mode:=velocity \
  max_velocity_command_rad_s:=0.2 max_velocity_rad_s:=0.5
```

另一个终端持续发送：

```bash
ros2 topic pub -r 20 \
  /rs03_current_torque/velocity_command_rad_s \
  std_msgs/msg/Float32 "{data: 0.1}"
```

`velocity_current_limit_a` 限制速度环可调用的电流，
`velocity_acceleration_rad_s2` 和 `velocity_slew_rate_rad_s2` 限制加速过程。

## PP 相对位置模式

位置命令是相对于节点启动位置的偏移量，不是绝对编码器位置。第一次测试建议
偏移 `0.05 rad`（约 2.9 度），最大偏移和速度分别限制为 `0.2 rad`、
`0.2 rad/s`：

```bash
ros2 launch rs03_current_torque_control rs03_current_torque.launch.py \
  auto_enable:=true control_mode:=position_pp \
  position_max_offset_rad:=0.1 position_current_limit_a:=0.3 \
  position_speed_limit_rad_s:=0.1 position_acceleration_rad_s2:=0.3 \
  position_tracking_error_rad:=0.25 max_velocity_rad_s:=0.3
```

另一个终端持续发送：

```bash
ros2 topic pub -r 20 \
  /rs03_current_torque/position_offset_command_rad \
  std_msgs/msg/Float32 "{data: 0.03}"
```

PP 模式启动后只会读取绝对计圈机械位置并进入 armed 状态，收到第一条有效位置
命令前保持失能。收到命令后，节点先用 `mechPos (0x7019)` 覆盖残留的旧
`loc_ref`，然后使能并按官方 PP 顺序重新写入限速、加速度和新位置目标。如果终端
一启动后电机自行移动，立即停机，不要继续发布命令。

正负方向必须先用小偏移确认。命令超时会停止驱动器，但不会发送“返回启动
位置”的命令。位置误差超过 `position_tracking_error_rad`、连续超速或过温均会
触发停机锁定。`position_current_limit_a` 默认把位置环可调用的电流限制为
`0.5 A`。

### WSL2 + SLCAN 兼容适配器

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
ros2 topic pub -r 20 /rs03_current_torque/current_command_a std_msgs/msg/Float32 "{data: 0.2}"
```

直接力矩模式命令（先把 YAML 的 `control_mode` 改成 `torque`）：

```bash
ros2 topic pub -r 20 /rs03_current_torque/torque_command_nm std_msgs/msg/Float32 "{data: 0.5}"
```

命令必须持续刷新；超过 `command_timeout_s`（默认 100 ms）后会归零、停机并锁定，
需要重启节点才能再次使能。节点退出时也会先发零命令，再发送停止帧。

读取单个反馈样本时使用 ROS2 自带的正常退出选项，不要用 Linux `timeout`
强制终止 ROS2 CLI：

```bash
ros2 topic echo --once /rs03_current_torque/estimated_torque_nm
```

## 与 PID 的关系

- 电流模式：ROS2 给 `iq_ref`，RS03 内部 PI 产生 PWM。
- 直接力矩模式：ROS2 给力矩前馈，驱动器换算并调用内部电流环。
- 若未来接入独立力矩传感器，才适合在 ROS2 中增加低频外部力矩 PI，其输出应为 `iq_ref`，且频率必须低于内部电流环。
