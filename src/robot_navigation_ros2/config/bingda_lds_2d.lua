-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Liu Fuzhi */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  -- use_odometry = false,                 -- [OLD] 纯激光,旋转/快速运动易丢轨
  use_odometry = true,                     -- [NEW] 接入 base_control_ros2 的 /odom 做先验
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,          -- 原 0.3, 仅 RViz 显示用
  pose_publish_period_sec = 0.02,           -- 50Hz map->odom, 高于 Nav2 20Hz 控制需求 (原 200Hz)
  trajectory_publish_period_sec = 0.1,      -- 10Hz, 仅 RViz 显示用 (原 33Hz)
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- RK3588 共 8 核: 默认 background 4 + Ceres 7 会与 MPPI 控制循环抢核,
-- 实测控制周期超时与后端优化批次明显重合, 限到 3 线程
MAP_BUILDER.num_background_threads = 3

-- ============================================================================
-- 前端 (Local SLAM) 参数
-- ============================================================================
-- TRAJECTORY_BUILDER_2D.min_range = 0.12       -- [OLD]
-- TRAJECTORY_BUILDER_2D.max_range = 3.5        -- [OLD] 人为限制太小,丢失远端特征
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.   -- [OLD]
TRAJECTORY_BUILDER_2D.min_range = 0.15          -- [NEW] 略提高避开雷达自身壳
TRAJECTORY_BUILDER_2D.max_range = 8.0           -- [NEW] S4 实测稳定量程
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0    -- [NEW] 与 max_range 一致

-- [P2-3] IMU quality excellent (gyro std 6e-4 rad/s, accel z=9.8 calibrated, zero-bias<1e-5)
-- Benefit: high-freq gyro constraint + gravity alignment, suppresses rotation drift
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- IMU gravity filter time constant (seconds), 10 is default, lower = trust accel more
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- 实时相关性扫描匹配 (CSM) 搜索窗口 (前端鲁棒性)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e1

-- Ceres 局部优化权重 (10/40 均为 Cartographer 默认值, 无需再调)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- 运动滤波: 三参数齐全, 静止时不插帧, 移动时按距离/角度采样
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- [OLD] 仅角度,过敏感
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5      -- [NEW]
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- [NEW]
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- [NEW] 放宽

-- 自适应体素滤波: 输入点云降采样,平衡精度与算力
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

-- 子图大小: 90 = 默认, 大空间可调到 120~160
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- ============================================================================
-- 后端 (Pose Graph / Loop Closure) 参数
-- ============================================================================
POSE_GRAPH.optimize_every_n_nodes = 90        -- 默认 90; 想更快闭环可降到 30~50
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2                -- 0.3→0.2: 降低后端约束计算峰值
POSE_GRAPH.constraint_builder.min_score = 0.65                    -- 局部约束门槛
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 全局闭环门槛
POSE_GRAPH.constraint_builder.max_constraint_distance = 8.0       -- 默认 15m, 当前室内场景数米级
-- 关闭逐约束日志 (实测单次运行 1000~2800 条 INFO, 拖慢 CPU 峰值与磁盘 I/O)
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- odometry 权重 1e5 均为 Cartographer 默认值; 若麦轮侧滑导致 odom 拉偏地图再考虑下调
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4  -- 默认 7, 限制优化批次抢占全部核心

return options
