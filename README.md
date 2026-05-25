# ESP32-S3 micro-ROS 机器人下位机驱动

## 简介
本项目是一个基于 ESP32-S3 和 micro-ROS 架构的两轮差速机器人下位机固件。配合上位机（例如 RDK X5 开发板）协同工作，负责底层的硬件控制与传感器数据采集。

上位机相关代码与工作空间请访问：[rdkrobot_ws](https://github.com/501Ranger/rdkrobot_ws)

本项目的配套硬件（包括 PCB 设计等）将会开源在 **立创开源广场 (OSHWHub)**，目前资料正在整理中，整理完成后将正式发布并在此处更新链接。

通过本项目，ESP32-S3 能够通过串口或 WiFi 建立与上位机的 micro-ROS 连接，实现实时的里程计计算、IMU（姿态传感器）数据采集以及双电机的 PID 闭环控制。

## 工程结构
```
esp32-micro-ros-driver/
├── include/                # 头文件目录
│   ├── encoder_reader.h    # 编码器脉冲读取与速度计算
│   ├── motor_driver.h      # 电机驱动控制接口 (高频 PWM)
│   ├── pid_controller.h    # 电机速度 PID 闭环控制算法
│   ├── qmi8658_sensor.h    # QMI8658 六轴 IMU 传感器驱动
│   ├── robot_app.h         # micro-ROS 节点、发布者、订阅者应用逻辑
│   ├── robot_config.h      # ⚠️ 机器人核心参数及硬件引脚配置文件
│   ├── robot_types.h       # 核心数据结构定义 (如状态机、传感器数据格式)
│   ├── web_manager.h       # 网页摇杆、WiFi 配网与 Web API
│   └── wifi_config_manager.h # WiFi/Agent 配置的 Flash 持久化
├── src/                    # 源代码目录
│   ├── encoder_reader.cpp
│   ├── main.cpp            # 主入口，处理硬件初始化与 micro-ROS 状态机
│   ├── motor_driver.cpp
│   ├── pid_controller.cpp
│   ├── qmi8658_sensor.cpp
│   ├── robot_app.cpp       # ROS 话题订阅(cmd_vel)与发布(odom, tf)的具体实现
│   ├── web_manager.cpp     # Web 服务、网页摇杆与配网接口
│   └── wifi_config_manager.cpp # NVS Flash 配置读写
└── platformio.ini          # ⚠️ PlatformIO 工程配置文件 (包含多环境编译配置)
```

## 主要功能
- **双路电机驱动与闭环控制**：实现两轮独立的速度 PID 闭环控制，平滑响应速度指令。
- **高频编码器读取**：实时读取双轮编码器数据，计算轮速及行驶距离。
- **IMU 传感器集成**：集成 QMI8658 六轴 IMU 传感器，提供精准的加速度、角速度及姿态信息。
- **micro-ROS 标准通信**：
  - **订阅** `cmd_vel` (`geometry_msgs/msg/Twist`)，接收上位机下发的速度和转向控制指令。
  - **发布** `odom` (`nav_msgs/msg/Odometry`)，向ROS 2网络提供基于轮式里程计的位置数据。
  - **发布** `/tf` 变换，发布 `odom` 到 `base_link` 的坐标系转换。
- **全新 Web 控制终端 & 网页摇杆**：基于 HTML5/CSS3/JS 全新设计的深色科技感网页控制端。除虚拟摇杆外，新增了 **E-Stop（紧急制动）安全按钮**、电池电量与控制状态面板，并支持 WebSocket 断线自动回连。
- **mDNS & NetBIOS 局域网解析**：设备连接网络后，会自动注册 `http://esp32robot.local/` (mDNS) 和 `http://esp32robot/` (NetBIOS)，无需记忆动态 IP 即可轻松访问控制页面。
- **多途径通信与 OTA 支持**：支持通过 **USB串口** 或 **WiFi** 与上位机 (Agent) 进行连接，并支持 **OTA (空中升级)** 无线刷写固件，极大方便调试。

## 详细教程

### 1. 开发环境准备
推荐使用 **VSCode + PlatformIO** 插件进行开发：
1. 下载并安装 [Visual Studio Code](https://code.visualstudio.com/)。
2. 在 VSCode 的扩展市场中搜索并安装 `PlatformIO IDE` 插件。
3. 克隆或下载本项目源码，使用 VSCode 打开 `esp32-micro-ros-driver` 文件夹。
4. 等待 PlatformIO 自动加载项目并下载工具链。

### 2. WiFi 与 Agent 配置
固件支持通过网页配置 WiFi，并将配置保存到 ESP32 的 NVS Flash 中。普通 USB 烧录或 OTA 更新固件不会清除这部分数据，因此后续升级后仍会自动使用上一次保存的 WiFi。

首次启动或 WiFi 连接失败时，ESP32 会开启配置热点：

```text
SSID: ESP32-Robot-Setup
密码: 12345678
配置页: http://192.168.4.1/wifi
```

在全新设计的深色科技感配置页中，你可以：
- 填写 **WiFi SSID**、**WiFi 密码**、**micro-ROS Agent IP** 以及 **Agent UDP 端口**。
- **历史记录管理器**：自动在浏览器本地缓存最近使用的最多 5 组配置记录，支持一键载入和快速切换，无需重复输入。
- **状态面板**：直观查看当前的 STA（局域网）连接状态与 IP，以及 AP 热点 IP。
- **清除配置**：提供“清除配置”按钮，确认后会擦除 NVS Flash 中的 WiFi 设定并重启设备，使其恢复到出厂默认状态。

保存配置后设备会自动重启，并尝试使用新配置连接局域网。连接成功后，访问设备的局域网 IP 仍可进入 `/wifi` 修改配置，也可以继续使用 OTA 无线烧录。

#### 访问控制终端与网页摇杆
当 ESP32 已连接到 WiFi，或设备处于配置热点模式下时，都可以通过浏览器访问 Web 控制页面。
得益于新增的域名解析服务，你**无需记忆多变的局域网 IP**，可直接使用以下本地域名进行访问：

- **摇杆终端页面**：
  - Windows / macOS / Linux / iOS / Android（同一局域网下）：
    `http://esp32robot.local/` (推荐) 或 `http://设备IP/`
  - Windows 系统（无 mDNS 支持时）：
    `http://esp32robot/` (NetBIOS 协议)
- **配网管理页面**：
  - `http://esp32robot.local/wifi` 或 `http://esp32robot/wifi` 或 `http://设备IP/wifi`

如果设备处于配置热点模式，默认地址是：
- 摇杆页面: `http://192.168.4.1/`
- 配网页面: `http://192.168.4.1/wifi`

网页摇杆通过 WebSocket 以约 20Hz 发送速度指令。向上推动摇杆控制前进/后退，左右推动控制转向；松手后页面会立即发送零速度指令。若浏览器断开或一段时间没有收到摇杆数据，固件会自动停车。

此外，Web 控制端集成了以下实用工具：
- **E-Stop 紧急刹车按钮**：页面中央醒目的红色 `EMERGENCY STOP` 按钮。按下后，机器人将立即制动并锁定输出（控制状态变为“急停触发”），虚拟摇杆失效，防止误触。再次点击可解除急停，恢复就绪状态。
- **设备状态监测**：实时显示当前电池电量、连接状态（已连接/未连接）及控制状态（就绪/急停触发），并支持断线自动重连。

`platformio.ini` 中的 `build_flags` 仍可作为首次烧录的默认/fallback 配置；如果 Flash 中已经保存过网页配置，则优先使用 Flash 中的配置。

打开项目根目录的 `platformio.ini`，找到 `build_flags` 部分：
```ini
build_flags = 
    -DWIFI_SSID=\"你的WiFi名称\"         # 可选：首次启动 fallback WiFi 名称
    -DWIFI_PASSWORD=\"你的WiFi密码\"      # 可选：首次启动 fallback WiFi 密码
    -DAGENT_IP=\"192.168.10.198\"       # 可选：fallback Agent IP
    -DAGENT_PORT=8888                   # 可选：fallback Agent 端口
```
> *注：除此之外，如果你的硬件引脚接线、电机参数（如轮径、编码器精度、减速比等）与默认值不一致，请在 `include/robot_config.h` 中进行相应的调整。*

### 3. 如何使用多版本编译环境 (`platformio.ini` 解析)
本项目在 `platformio.ini` 中配置了四种不同的编译环境 (`env`)，以便你在不同场景下无缝切换。

| 环境名 (`[env:xxx]`) | 通信方式 | 固件烧录方式 | 适用场景 |
| --- | --- | --- | --- |
| `esp32-s3-serial` | **USB串口** | USB 有线烧录 | 初期有线调试，直接通过 USB 线供电与通信。 |
| `esp32-s3-wifi` | **WiFi** | USB 有线烧录 | 测试 WiFi 通信，但设备就在手边，通过有线烧录。 |
| `esp32-s3-serial-ota` | **USB串口** | 局域网 OTA 无线烧录 | 机器人已组装，USB 用于通信，但希望通过无线烧录更新代码。 |
| `esp32-s3-wifi-ota` | **WiFi** | 局域网 OTA 无线烧录 | **全无线模式**：通信与固件更新全部在局域网内无线完成。 |

**如何切换和使用？**
你可以通过修改 `platformio.ini` 第一部分中的 `default_envs` 来指定默认编译环境：
```ini
[platformio]
default_envs = esp32-s3-serial  ; 例如：切换为普通串口通信+有线烧录版本
```
或者，你也可以在 VSCode 底部 PlatformIO 状态栏中，点击当前的环境名称，在弹出的列表中手动选择你需要的环境。

*注意：如果你选择使用带有 `-ota` 后缀的环境，请确保将对应环境下的 `upload_port` 参数修改为该 ESP32 设备目前在局域网内被分配到的 IP 地址。*

### 4. 编译与运行
1. **编译**：点击 VSCode 底部状态栏的 **`Build`** (✓ 图标)。首次编译时会自动拉取 `micro_ros_platformio` 等依赖库，可能需要一些时间，请耐心等待。
2. **烧录**：编译成功后，使用数据线连接 ESP32-S3，点击 **`Upload`** (→ 图标) 进行烧录。
3. **启动上位机 Agent**：
   在你的上位机 (例如 RDK X5) 上，启动相应的 micro-ROS Agent。
   - 如果你使用的是串口通信环境：
     ```bash
     docker run -it --rm -v /dev:/dev --privileged microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 921600 -v6
     ```
   - 如果你使用的是 WiFi 通信环境：
     ```bash
     docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
     ```
4. **验证连接**：当下位机成功连接到 Agent 后，在上位机终端执行：
   ```bash
   ros2 node list
   ```
   如果出现 `/esp32s3_base` 节点，则证明下位机已成功接入 ROS 2 系统！你可以开始监听 `/odom` 话题或下发 `/cmd_vel` 速度控制了。
5. **网页遥控**：在手机或电脑浏览器中打开 `http://esp32robot.local/` 或 `http://设备IP/`，即可使用全新设计的控制终端和网页摇杆临时接管底盘运动。控制页面支持紧急刹车（E-Stop）、实时电量指示，停止操作或关闭页面后会自动停车。
