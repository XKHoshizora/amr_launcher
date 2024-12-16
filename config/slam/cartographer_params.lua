include "map_builder.lua"
include "trajectory_builder.lua"

--------------------------------------------------
-- 当前配置适用于：
-- 1. 使用 RPLiDAR S2
--    - 测量角度：360°，但是由于雷达安装位置的后方有遮挡，只需获取正前方左右共200°范围内的数据
--    - 测量范围：0.05m-30m
--    - 采样率：32000 Hz
--    - 扫描频率：10Hz
--    - 角分辨率：0.1125°
--    - 精度：±30mm
--    - 距离分辨率：13mm
-- 2. 使用轮式里程计
--    - 数据发布频率：20Hz
-- 3. 使用IMU
-- 4. 不使用GPS
--------------------------------------------------

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 1,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 2D SLAM和后台线程设置
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4  -- 推荐的后台线程数，可根据设备计算能力调整

-- 2D前端配置
TRAJECTORY_BUILDER_2D.min_range = 0.05  -- RPLiDAR S2的最小测距
TRAJECTORY_BUILDER_2D.max_range = 25.0  -- RPLiDAR S2的最大测距
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 每个子地图的最大扫描数目

POSE_GRAPH.optimization_problem.huber_scale = 1e1  -- 控制优化鲁棒性，减少异常值的影响

return options