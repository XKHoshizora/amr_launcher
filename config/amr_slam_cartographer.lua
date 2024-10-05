include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",  -- 追踪 IMU 或机器人的主框架
  published_frame = "base_link",
  odom_frame = "odom",  -- 里程计数据的帧
  provide_odom_frame = false,  -- Cartographer 不提供 odom
  use_odometry = true,  -- 启用里程计数据
  use_imu_data = false,  -- 启用 IMU 数据
  num_laser_scans = 1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.3,
  trajectory_publish_period_sec = 0.3,
}

TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,  -- 启用 IMU 数据
  min_range = 0.15,  -- RPLiDAR S2 的最小测距范围
  max_range = 30.0,  -- RPLiDAR S2 的最大测距范围
  num_accumulated_range_data = 1,
  
  -- 匹配器配置，确保传感器数据与地图一致
  ceres_scan_matcher = {
    occupied_space_weight = 1.0,
    translation_weight = 10.0,
    rotation_weight = 40.0,
  },
}

MAP_BUILDER.use_trajectory_builder_

2d = true
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7