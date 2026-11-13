#!/usr/bin/env bash
# 干净复刻验收：编译、测试、启动文件检查。不启动底盘或雷达。
set -euo pipefail

cd "$(dirname "$0")/.."

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: 未找到 /opt/ros/humble，请先安装 ROS 2 Humble" >&2
  exit 1
fi

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_setup /opt/ros/humble/setup.bash

for pkg in base_control_ros2 nanoomni_description robot_navigation_ros2 ydlidar_ros2_driver; do
  if [ ! -f "src/${pkg}/package.xml" ] && [ ! -f "src/${pkg}/src/package.xml" ]; then
    echo "ERROR: 缺少软件包 ${pkg}" >&2
    exit 1
  fi
done

echo "[1/4] 编译"
colcon build --symlink-install

echo "[2/4] 加载工作空间"
source_setup install/setup.bash

echo "[3/4] 运行测试"
colcon test --packages-select robot_navigation_ros2
result="$(colcon test-result --verbose | tail -n 1)"
echo "$result"
if ! echo "$result" | grep -Eq "0 errors, 0 failures"; then
  echo "ERROR: 测试未通过" >&2
  exit 1
fi

echo "[4/4] 检查启动文件"
ros2 launch robot_navigation_ros2 slam_nav.launch.py --show-args >/dev/null
ros2 launch robot_navigation_ros2 nav_with_map.launch.py --show-args >/dev/null
ros2 launch robot_navigation_ros2 web_control.launch.py --show-args >/dev/null

echo "REPRODUCE CHECK PASSED"
