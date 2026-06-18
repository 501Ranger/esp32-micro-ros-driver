# ESP32-S3 micro-ROS 两轮差速机器人

基于 **ESP32-S3 + micro-ROS** 的两轮差速机器人，提供电机闭环控制、IMU 数据采集、轮式里程计计算以及 Web 遥控等功能。通过串口或 WiFi 与 ROS 2 上位机通信。

> **配套项目**
> - 上位机工作空间：[rdkrobot_ws](https://github.com/501Ranger/rdkrobot_ws)
> - 配套硬件 PCB（整理中）：即将开源于 [立创开源广场 (OSHWHub)](https://oshwhub.com/)

## 功能特性

| 类别 | 说明 |
|------|------|
| **电机控制** | 双路 20 kHz PWM 驱动 + 位置式 PID 闭环 + 前馈补偿，支持占空比斜坡限速 |
| **编码器** | 硬件中断正交解码，1040 脉冲/转，实时轮速计算 |
| **IMU** | QMI8658C 六轴传感器（I2C），提供加速度 / 角速度 / 姿态数据 |
| **里程计** | 差速运动学模型实时积分，发布 `odom` + `tf` |
| **电池监测** | ADC 电压采集 + 分压比换算，低电量 LED 闪烁告警，发布 `battery_state` |
| **Web 控制台** | 网页摇杆、实时状态面板、WebSocket 断线重连 |
| **WiFi 配网** | AP 热点配置门户，NVS Flash 持久化，浏览器端历史记录管理 |
| **局域网解析** | mDNS (`esp32robot.local`) + NetBIOS (`esp32robot`)，免 IP 访问 |
| **OTA 升级** | 局域网无线固件烧录 |
| **调试工具** | VOFA+ JustFloat 波形输出（Serial1），串口 CLI 动态 PID 调参 |

## 工程结构

```
esp32-micro-ros-driver/
├── include/
│   ├── robot_config.h          # 引脚定义与物理参数配置
│   ├── robot_types.h           # 核心数据结构与状态枚举
│   ├── encoder_reader.h        # 编码器驱动
│   ├── motor_driver.h          # 电机 PWM 驱动
│   ├── pid_controller.h        # PID 闭环控制器
│   ├── qmi8658_sensor.h        # IMU 传感器驱动
│   ├── robot_app.h             # 应用核心协调类
│   ├── web_manager.h           # Web 服务与 WebSocket
│   ├── web_pages.h             # HTML/CSS/JS 静态资源
│   └── wifi_config_manager.h   # NVS 配置读写
├── src/                        # 对应源文件实现
│   └── main.cpp                # Arduino 入口 (setup / loop)
├── test/
│   └── joystick.html           # 网页摇杆设计稿
└── platformio.ini              # PlatformIO 工程配置
```

## 快速开始

### 环境要求

- [VSCode](https://code.visualstudio.com/) + [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) 插件
- USB 数据线（首次烧录）
- ESP32-S3 开发板 + 配套驱动底板

### 1. 克隆与打开

```bash
git clone https://github.com/501Ranger/esp32-micro-ros-driver.git
# 使用 VSCode 打开项目文件夹，PlatformIO 将自动加载依赖
```

### 2. 配置参数

如果你的硬件引脚或机器人参数与默认值不同，请修改 [`include/robot_config.h`](include/robot_config.h)：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `WHEEL_DIAMETER_M` | 0.048 m | 轮径 |
| `TRACK_WIDTH_M` | 0.130 m | 轮距 |
| `ENCODER_TICKS_PER_WHEEL_REV` | 1040 | 编码器每转脉冲数 |
| `PWM_FREQUENCY` | 20000 Hz | 电机 PWM 频率 |
| `MOTOR_PID_KP / KI / KD` | 0.55 / 2.66 / 0.00 | PID 增益 |

`platformio.ini` 中可设置 fallback 网络参数（Flash 中有保存的配置时优先使用 Flash 配置）：

```ini
build_flags =
    -DWIFI_SSID=\"你的WiFi\"
    -DWIFI_PASSWORD=\"你的密码\"
    -DAGENT_IP=\"192.168.1.1\"
    -DAGENT_PORT=8888
```

### 3. 编译与烧录

点击 VSCode 底部状态栏 **Build** (✓) 编译，**Upload** (→) 烧录。

> 首次编译将自动拉取 `micro_ros_platformio` 等依赖库，耗时较长。

### 4. WiFi 配网

首次启动或 WiFi 连接失败时，设备开启配置热点：

| 项目 | 值 |
|------|----|
| SSID | `ESP32-Robot-Setup` |
| 密码 | `12345678` |
| 配置页 | `http://192.168.4.1/wifi` |

在配置页面填写 WiFi 信息和 Agent 地址后保存，设备自动重启并连接网络。

### 5. 启动上位机 Agent

```bash
# 串口通信
docker run -it --rm -v /dev:/dev --privileged \
  microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 921600 -v6

# WiFi 通信
docker run -it --rm --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888
```

### 6. 验证

```bash
ros2 node list          # 应出现 /esp32s3_base
ros2 topic echo /odom   # 查看里程计数据
```

## 编译环境

项目在 `platformio.ini` 中定义了 4 个编译环境，按需切换：

| 环境名 | 通信方式 | 烧录方式 | 说明 |
|--------|----------|----------|------|
| `esp32-s3-serial` | USB 串口 | 有线 | 有线调试首选 |
| `esp32-s3-wifi` | WiFi UDP | 有线 | 测试无线通信 |
| `esp32-s3-serial-ota` | USB 串口 | OTA 无线 | 已组装设备，串口通信 + 无线更新 |
| `esp32-s3-wifi-ota` | WiFi UDP | OTA 无线 | 全无线模式 |

切换方式：修改 `platformio.ini` 中 `default_envs`，或在 VSCode 底部状态栏选择。

> 使用 OTA 环境时，需将 `upload_port` 设为设备当前的局域网 IP 或 `esp32robot.local`。

## ROS 2 接口

### 节点

`/esp32s3_base`

### 话题

| 话题 | 类型 | 方向 | 频率 | 说明 |
|------|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | — | 速度指令，100 ms 超时自动停车 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 50 Hz | 轮式里程计（含协方差） |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | 50 Hz | `odom` → `base_link` 变换 |
| `/battery_state` | `sensor_msgs/BatteryState` | 发布 | 1 Hz | 电池电压与电量百分比 |

### 坐标系

- `odom` — 里程计参考系
- `base_link` — 机器人本体坐标系

## Web 控制台

设备连接 WiFi 后，通过浏览器访问控制页面：

| 页面 | 地址 |
|------|------|
| 摇杆控制台 | `http://esp32robot.local/` |
| WiFi 配网 | `http://esp32robot.local/wifi` |

> Windows 无 mDNS 时可使用 `http://esp32robot/`（NetBIOS）。

**控制优先级**：网页摇杆 > `cmd_vel` 话题。E-Stop 触发时锁定所有输出，需手动解除。

## 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| [micro_ros_platformio](https://gitee.com/ohhuo/micro_ros_platformio.git) | — | micro-ROS 客户端 |
| [ESPAsyncWebServer](https://github.com/mathieucarbou/ESPAsyncWebServer) | ≥ 3.3.23 | 异步 Web 服务 |
| [ArduinoJson](https://github.com/bblanchon/ArduinoJson) | ≥ 6.21.3 | JSON 解析 |

平台：`espressif32` / `arduino` / ROS 2 `humble`

## 硬件引脚参考

<details>
<summary>点击展开默认引脚映射</summary>

| 功能 | 引脚 |
|------|------|
| 左电机 IN1 / IN2 | GPIO 14 / 21 |
| 左编码器 A / B | GPIO 11 / 10 |
| 右电机 IN1 / IN2 | GPIO 48 / 47 |
| 右编码器 A / B | GPIO 12 / 13 |
| I2C SDA / SCL | GPIO 35 / 36 |
| UART1 TX / RX | GPIO 16 / 15 |
| Serial2 TX / RX | GPIO 1 / 2 |
| CAN TX / RX | GPIO 6 / 7 |
| 电池 ADC | GPIO 18 |
| 低电量 LED | GPIO 39 |
| 蜂鸣器 | GPIO 38 |
| 按钮 LED | GPIO 37 |
| 按钮 | GPIO 8 |

</details>

## 许可证

待添加。
