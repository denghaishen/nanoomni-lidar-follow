# NanoOmni 激光跟随机器人（Cartographer + Nav2 + 网页圈选）

基于 ROS 2 Humble 的室内小型全向轮机器人：
**免预建图、免手动初始位姿**，在手机/电脑浏览器里拖一个圆圈选中目标，机器人保持约 1 米距离自主跟随；人走它走、人停它停、人靠近它主动后退。

![状态](https://img.shields.io/badge/ROS2-Humble-blue) ![测试](https://img.shields.io/badge/tests-43%20passed-brightgreen)

---

## 1. 功能特性

- **在线 SLAM 导航**：Cartographer 实时建图，开机位置即地图原点，不需要 RViz 2D Pose Estimate。
- **网页圈选跟随**：浏览器拖拽圆形区域选择目标，无需安装 App。
- **双向距离保持**：目标远离→前进，目标靠近→主动倒车，距离死区内静止。
- **转向优先**：目标偏离正前方超过约 17° 时先转向再移动。
- **距离可调**：网页滑杆实时调整保持距离，范围 0.4–2.0 m，默认 1.0 m。
- **分层安全**：模式仲裁 → 速度平滑 → 激光/TF 失效看门狗 → 前方停止区 → 底盘。
- **一键启停**：网页按钮启动/停止整套底盘、雷达、建图与导航；网页常驻不中断。
- **电话/电脑自适应**：同一局域网内手机、平板、电脑均可访问。
- **有图模式可选**：需要复用固定地图时切换 `nav_with_map.launch.py` + AMCL。

---

## 2. 系统架构

```text
手机/电脑浏览器
   │  HTTP 静态页面 + WebSocket(JSON, 5 Hz)
   ▼
lidar_follow_web ──► /follow/selection (PolygonStamped, base_link)
   │                       │
   │                       ▼
   │               lidar_target_follower ──► /cmd_vel_follow
   │                       ▲
   │                       │ 激光点云 + TF
   │                    /scan
   ▼
motion_command_arbiter (NAV / FOLLOW / STOP)
   │  /cmd_vel_nav ◄── Nav2 controller / behavior
   ▼
/cmd_vel_selected
   ▼
velocity_smoother
   ▼
/cmd_vel_smoothed
   ▼
scan_safety_watchdog        (扫描或 TF 超时 → 强制零速)
   ▼
/cmd_vel_collision_input
   ▼
collision_monitor           (FrontStopZone 仅 NAV 模式生效)
   ▼
/cmd_vel
   ▼
base_control_ros2 ──► 串口 /dev/move_base ──► 底盘
```

关键设计：

- 所有速度源都必须经过同一条安全链，网页节点**没有**直接发布 `/cmd_vel` 的能力。
- `motion_command_arbiter` 切换模式时先发零速，并要求新速度源产生新指令后才放行。
- 跟随模式会通过参数服务临时关闭 `FrontStopZone`，使“目标靠近时倒车”不被前方停止区拦截；退回 NAV 时自动重新启用。
- `scan_safety_watchdog` 修复了 Humble 原生 Collision Monitor 在扫描超时后继续透传速度的问题。

---

## 3. 硬件与软件环境

| 项目 | 要求 |
|---|---|
| 计算平台 | RK3588 / 树莓派 4B 及以上（实测 Orange Pi 5 Pro） |
| 系统 | Ubuntu 22.04 + ROS 2 Humble |
| 底盘 | 冰达 NanoOmni 系列（实测 4WD，固件 1.8.2） |
| 雷达 | YDLIDAR S4（2D，`/dev/lidar`，115200） |
| 底盘串口 | `/dev/move_base`，115200 |
| 显示 | 可选 RViz；日常使用只需要浏览器 |

### 依赖安装

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-velocity-smoother \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  python3-serial
```

可选：自动补齐 `package.xml` 中声明的依赖（首次使用 rosdep 时先执行 `sudo rosdep init && rosdep update`）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 串口权限

```bash
sudo usermod -aG dialout "$USER"
```

确保存在以下设备名（可用 udev 规则按 USB 串号固定）：

```text
/dev/move_base
/dev/lidar
```

---

## 4. 编译

```bash
cd ~/cartographer_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  base_control_ros2 ydlidar_ros2_driver nanoomni_description robot_navigation_ros2
source install/setup.bash
```

---

## 5. 运行

### 5.1 无地图导航 + 跟随（推荐）

不需要预先建图，也不需要标初始位姿：

```bash
ros2 launch robot_navigation_ros2 slam_nav.launch.py use_rviz:=true
```

### 5.2 有地图导航（复用固定地图）

```bash
ros2 launch robot_navigation_ros2 nav_with_map.launch.py \
  map:=$HOME/map/my_map.yaml \
  use_rviz:=true
```

启动后必须在 RViz 用 **2D Pose Estimate** 标出机器人真实位置，并让箭头指向真实车头；确认红色激光点与地图墙体重合后再导航。

### 5.3 网页端

```bash
ros2 launch robot_navigation_ros2 web_control.launch.py
```

浏览器打开：

```text
http://<机器人IP>:8080
```

使用流程：**启动系统 → 等待红色激光点 → 在人物位置拖拽圆圈 → 松开自动跟随 → 停止跟随/停止系统**。

### 5.4 开机常驻（可选）

```bash
mkdir -p ~/.config/systemd/user
cp src/robot_navigation_ros2/systemd/robot-web-control.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robot-web-control.service
loginctl enable-linger "$USER"      # 未登录桌面也能启动
```

服务使用 `ROS_LOCALHOST_ONLY=1`，ROS 2 的 DDS 流量限制在本机回环，避免与手机 Wi-Fi 抢带宽。
如果需要**其他电脑上的 RViz** 看到话题，请删除该环境变量并自行评估网络负载。

---

## 6. 关键参数

配置文件：`src/robot_navigation_ros2/config/nav2_params.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `follow_distance` | `1.0 m` | 保持距离，可在网页实时调整 |
| `distance_deadband` | `0.05 m` | 距离死区 |
| `max_forward_speed` | `0.30 m/s` | 最大前进速度 |
| `max_reverse_speed` | `0.20 m/s` | 最大后退速度（低于前进速度） |
| `max_angular_speed` | `1.0 rad/s` | 最大角速度 |
| `rotate_first_angle` | `0.30 rad` | 超过该偏角先转向，不前进 |
| `angular_deadband` | `0.10 rad` | 角度死区 |
| `target_radius` | `0.30 m` | 目标点云跟踪半径 |
| `min_target_points` | `1` | 允许人体稀疏单点回波 |
| `scan_timeout` | `0.5 s` | 激光失效判定时间 |
| `FrontStopZone` | 前缘 0.41 m / 宽 0.50 m | 仅 NAV 模式硬停 |

---

## 7. 测试

```bash
colcon test --packages-select robot_navigation_ros2
colcon test-result --verbose
```

覆盖内容：双向跟随、转向优先、角度/距离死区、目标丢失恢复、NAV/FOLLOW/STOP 仲裁、前方停止区几何、激光超时停车、网页圈选、距离调节、一键启停、进程监管。结果：**43 passed**。

---

## 8. 复刻验收（10 分钟）

前提：目标机器已安装 ROS 2 Humble、Nav2、Cartographer（见第 3 节）。

```bash
git clone --depth 1 https://github.com/denghaishen/nanoomni-lidar-follow.git
cd nanoomni-lidar-follow
bash tools/reproduce_check.sh
```

预期输出：

```text
Summary: 4 packages finished
43 tests, 0 errors, 0 failures, 0 skipped
REPRODUCE CHECK PASSED
```

实测耗时（RK3588，依赖已安装）：克隆约 1.1 分钟，编译约 18 秒，测试约 6 秒。

> 从全新 Ubuntu 开始，apt 安装 ROS/Nav2/Cartographer 通常需要 5–20 分钟，总时长会超过 10 分钟。“10 分钟复刻”适用于已具备 ROS 环境的机器。

脚本只做编译、测试和启动文件检查，**不会启动底盘、雷达或发送运动指令**。硬件联调需要真实机器人、`/dev/move_base`、`/dev/lidar` 和串口权限。

---

## 9. 已知限制

- 激光只识别几何轮廓，不做身份识别；旁边有相似物体时可能跟错。
- 跟随距离与速度按小型室内机器人设计，不适合高速或复杂室外场景。
- 网页没有登录验证，只应部署在可信局域网，不要映射到公网。
- 地图与航线文件包含环境坐标，公开前请确认是否涉及隐私。
- 底盘低压报警由固件负责，本软件不会屏蔽安全报警。
- 跟随模式不启用车尾防撞，现场需保持车尾空间。

---

## 10. 目录结构

```text
cartographer_ws/
├── src/
│   ├── base_control_ros2/          # 底盘串口驱动、里程计、IMU、电池
│   ├── ydlidar_ros2_driver/        # 雷达驱动
│   ├── nanoomni_description/       # URDF 与模型
│   └── robot_navigation_ros2/      # 本项目核心
│       ├── robot_navigation_ros2/  # 跟随、仲裁、看门狗、网页服务
│       ├── launch/                 # slam_nav / nav_with_map / nav2 / web_control
│       ├── config/                 # nav2_params.yaml、Cartographer、航线
│       ├── rviz/                   # 导航与建图配置
│       ├── web/                    # 手机/电脑网页
│       ├── systemd/                # 常驻服务模板
│       └── test/                   # 43 项自动化测试
├── tools/reproduce_check.sh        # 干净复刻验收脚本
└── .gitignore
```

---

## 11. 第三方组件与许可

| 组件 | 来源 | 许可 |
|---|---|---|
| `nanoomni_description` | 冰达机器人 | BSD（见 `package.xml`） |
| `ydlidar_ros2_driver` | YDLIDAR | MIT（见 `package.xml`） |
| `base_control_ros2` | 冰达机器人官方仓库 | 未声明，公开前请向厂商确认 |
| Cartographer / Nav2 | ROS 生态 | Apache-2.0 |

本仓库未附带地图文件（`.pgm`）与学习资料 `cartographer_study/`，请自行建图或从官方仓库获取。
