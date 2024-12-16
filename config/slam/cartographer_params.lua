include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                -- SLAM生成的全局地图坐标系名称
  tracking_frame = "base_link",     -- 机器人车体或IMU所在坐标系，一般为底盘坐标系
  published_frame = "base_link",    -- 发布位姿用的坐标系，通常与tracking_frame相同
  odom_frame = "odom",              -- 里程计（如果有）坐标系名称
  provide_odom_frame = false,       -- 是否由Cartographer提供odom坐标系，一般false
  use_odometry = true,              -- 是否使用外部里程计信息
  use_nav_sat = false,              -- 是否使用GPS等NavSat数据
  use_landmarks = false,            -- 是否使用地标信息
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,     -- 是否使用姿态外推器提升轨迹稳定性
  pose_extrapolator = {
    -- 姿态外推器的相关参数优化
  },
  publish_tracked_pose = true,
  published_pose_pipeline = {
    -- 用于发布pose的后处理管线（如滤波器）设置
  },
  publish_to_tf = true,             -- 是否通过tf发布坐标变换
  tf_publication_period_sec = 0.05, -- tf发布周期
  submap_publish_period_sec = 0.3,  -- 子图发布周期
  pose_publish_period_sec = 0.05,   -- 位姿发布周期
  trajectory_publish_period_sec = 0.05, -- 轨迹发布周期
  rangefinder_sampling_ratio = 1.0, -- 激光雷达数据采样比
  odometry_sampling_ratio = 1.0,    -- 里程计数据采样比
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,         -- IMU数据采样比
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 选择2D SLAM构建器

TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,                      -- 2D下是否使用IMU，一般2D地图可不用IMU
  min_range = 0.1,                           -- 激光雷达最小有效范围(m)
  max_range = 25.0,                          -- 激光雷达最大有效范围(m)
  missing_data_ray_length = 25.0,            -- 当某方向无数据时虚拟ray长度
  use_online_correlative_scan_matching = true,-- 是否使用在线相关扫描匹配（可改善短期定位）
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,
    angular_search_window = math.rad(5.0),
    translation_delta_cost_weight = 10,
    rotation_delta_cost_weight = 1e-1,
  },
  ceres_scan_matcher = {
    occupied_space_weight = 1,
    translation_weight = 5,
    rotation_weight = 1,
    ceres_solver_options = {
      max_num_iterations = 20,
    },
  },
  motion_filter = {
    max_time_seconds = 0.3,                  -- 去除运动幅度过小的帧，减少计算量
    max_distance_meters = 0.1,
    max_angle_radians = math.rad(1.0),
  },
  submaps = {
    resolution = 0.05,                       -- 子图分辨率(m/格)
    num_range_data = 45,                     -- 构成一个子图所需的激光雷达帧数
    grid_type = "PROBABILITY_GRID",          -- 栅格类型，可选PROBABILITY_GRID或TSDF
    range_data_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },
  min_score = 0.55,                          -- 全局匹配最低得分阈值
  global_sampling_ratio = 0.003,             -- 全局定位尝试的采样比例
  constraint_builder = {
    sampling_ratio = 0.3,                    -- 回环检测采样率
    min_score = 0.55,                        -- 回环约束的最低得分
    global_localization_min_score = 0.6,
  },
}

MAP_BUILDER.num_background_threads = 4         -- 后台线程数，越多CPU资源越足的情况下可加快优化
POSE_GRAPH = {
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    ceres_solver_options = {
      max_num_iterations = 40,
    },
  },
  optimize_every_n_nodes = 120,                -- 每多少个节点进行一次全局优化
  constraint_builder = {
    sampling_ratio = 0.3,
    min_score = 0.55,
    global_localization_min_score = 0.6,
  },
}

return options
