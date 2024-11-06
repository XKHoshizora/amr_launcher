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
-- 3. 暂未使用IMU
-- 4. 不使用GPS
--------------------------------------------------

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  --------------------------------------------------
  -- 1. 坐标系统配置
  --------------------------------------------------
  map_frame = "map",
  tracking_frame = "base_link",  -- 当使用IMU时改为"imu_link"
  published_frame = "odom",  -- 当没有外部里程计时，改为"base_link"
  odom_frame = "odom",  -- 当使用外部里程计时，该参数无效
  provide_odom_frame = false,  -- 当没有外部里程计时改为true，会产生虚拟里程计数据
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,

  --------------------------------------------------
  -- 2. 传感器使能配置
  --------------------------------------------------
  use_odometry = true,
  use_nav_sat = false,  -- 添加GPS时改为true
  use_landmarks = false,  -- 使用路标时改为true

  --------------------------------------------------
  -- 3. 传感器话题配置
  --------------------------------------------------
  -- RPLiDAR S2扫描频率为10Hz，不需要subdivision
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 4,  -- 快速移动场景（>0.5m/s）建议值：2-4
  num_point_clouds = 0,

  --------------------------------------------------
  -- 4. 时间和发布周期配置
  --------------------------------------------------
  -- 由于激光10Hz的扫描频率，适当调整发布周期
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2,  -- 200Hz，可以考虑改为0.02-0.05（20-50Hz）减少计算负担
  trajectory_publish_period_sec = 30e-3,

  --------------------------------------------------
  -- 5. 传感器数据采样比例配置
  --------------------------------------------------
  rangefinder_sampling_ratio = 1.,  -- 由于扫描频率较低，保持完整采样
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--------------------------------------------------
-- 6. SLAM后端配置
--------------------------------------------------
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

--------------------------------------------------
-- 7. 2D SLAM前端配置
--------------------------------------------------
-- 在进行扫描匹配之前需要累积的激光雷达扫描帧数
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
-- 子地图参数，每个子图使用多少帧数据构建
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
-- 考虑到13mm的距离分辨率，可设置较小的栅格分辨率（>13mm），默认为0.05m
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
-- 提高击中概率，因为前向数据更可靠
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

-- RPLiDAR S2激光雷达参数
TRAJECTORY_BUILDER_2D.min_range = 0.05  -- 最小量程0.05m
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- 最大量程30m
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.

-- 考虑到高精度的角分辨率(0.1125°)和距离精度(±30mm)
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 使用IMU时改为true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关性扫描匹配算法（Online Correlative Scan Matching）进行激光雷达数据的匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 由于激光精度较高，提高优化权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50.

-- voxel滤波参数，根据距离分辨率设置
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- 25mm的体素大小

--------------------------------------------------
-- 8. 闭环检测和优化配置
--------------------------------------------------
-- 考虑到10Hz的扫描频率，适当调整优化频率
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.constraint_builder.min_score = 0.75
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 由于激光精度较高，提高优化权重
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1e2
-- 提高局部SLAM权重，因为前向数据更可靠
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
-- 适当降低里程计权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2

return options