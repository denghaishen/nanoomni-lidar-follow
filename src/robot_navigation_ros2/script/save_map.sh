#!/usr/bin/env bash
# =============================================================================
# save_map.sh - 一键保存当前 Cartographer 在线 SLAM 地图为 nav2 兼容 .pgm/.yaml
# =============================================================================
# 用法:
#     bash ~/cartographer_ws/src/robot_navigation_ros2/script/save_map.sh           # 默认 ~/map/my_map
#     bash ~/cartographer_ws/src/robot_navigation_ros2/script/save_map.sh office    # ~/map/office
#     bash ... save_map.sh /tmp/foo                                                 # /tmp/foo.{pgm,yaml}
#
# 前提: 已经运行 slam_nav.launch.py (或至少 cartographer.launch.py),
#       /map 话题正在以 latched (Transient Local) QoS 持续发布.
#
# 原理: 调用 nav2_map_server 的 map_saver_cli, 订阅 /map 拿到当前占据栅格,
#       写入 PGM (image) + YAML (resolution / origin / 阈值) 两个文件.
# =============================================================================

set -e

# 解析路径 (默认 ~/map/my_map, 不带后缀)
DEFAULT_BASE="$HOME/map/my_map"
TARGET="${1:-$DEFAULT_BASE}"

# 如果用户只给了名字 (不含 /), 拼到 ~/map/ 下
case "$TARGET" in
    /*) ;;                              # 绝对路径, 不动
    */*) ;;                             # 相对路径包含 /, 不动
    *)  TARGET="$HOME/map/$TARGET" ;;   # 纯名字 -> ~/map/<name>
esac

# 确保目录存在
mkdir -p "$(dirname "$TARGET")"

echo "[save_map] saving to: ${TARGET}.pgm  +  ${TARGET}.yaml"
echo "[save_map] make sure /map is being published (slam_nav.launch.py running)..."

# 调用 nav2 map_saver_cli
#   -f <basename> : 输出文件名 (无后缀)
#   --ros-args -p save_map_timeout:=10.0 : 等待 /map 的超时 (秒，必须是浮点数)
#   --ros-args -p map_subscribe_transient_local:=true : ⭐ Cartographer 是 latched 必须用 TL
ros2 run nav2_map_server map_saver_cli \
    -f "$TARGET" \
    --ros-args \
    -p save_map_timeout:=10.0 \
    -p map_subscribe_transient_local:=true \
    -p free_thresh_default:=0.25 \
    -p occupied_thresh_default:=0.65

echo "[save_map] done. Files:"
ls -lh "${TARGET}.pgm" "${TARGET}.yaml" 2>/dev/null || true

echo
echo "[save_map] 后续启动有图导航:"
echo "    ros2 launch robot_navigation_ros2 nav_with_map.launch.py map:=${TARGET}.yaml"
