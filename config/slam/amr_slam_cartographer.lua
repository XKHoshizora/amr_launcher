-- 引入 Cartographer 的 map_builder 模块，它定义了地图的构建方式（2D/3D）
include "map_builder.lua"
-- 引入 trajectory_builder 模块，用于定义机器人轨迹的构建方法
include "trajectory_builder.lua"

-- 配置选项的定义
options = {
  -- 指定使用的地图构建器，这里使用的是通过 map_builder.lua 导入的 MAP_BUILDER
  map_builder = MAP_BUILDER,
  
  -- 指定使用的轨迹构建器，通过 trajectory_builder.lua 导入的 TRAJECTORY_BUILDER
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 地图坐标系的名称，所有的位姿和激光雷达数据都会以这个坐标系为参考
  map_frame = "map",
  
  -- 追踪坐标系，用于跟踪传感器或机器人的移动。通常是激光雷达或 IMU 的坐标系
  tracking_frame = "base_link",  -- 需根据机器人的坐标系设置
  
  -- 发布的坐标系的名称，通常是与 tracking_frame 相同的坐标系
  published_frame = "base_link",
  
  -- 里程计坐标系的名称，用于融合里程计数据。如果使用了 odom，则表示机器人的估计位姿
  odom_frame = "odom",
  
  -- 是否提供 odom 坐标系。设置为 true 表示 Cartographer 将生成一个 odom 参考框架
  provide_odom_frame = true,
  
  -- 是否将坐标系投影到 2D 平面。对于 2D SLAM，此设置通常为 false
  publish_frame_projected_to_2d = false,
  
  -- 是否使用位姿预测器进行位姿外推。如果传感器频率较慢或存在延迟，可以启用此功能
  use_pose_extrapolator = true,
  
  -- 是否使用里程计数据。如果机器人提供 odom 数据，将其设置为 true
  use_odometry = true,
  
  -- 是否使用 GPS 数据。如果有 GPS 数据可用，可以设置为 true（不使用时为 false）
  use_nav_sat = false,
  
  -- 是否使用地标数据（如视觉标志）。如果没有地标数据，设置为 false
  use_landmarks = false,
  
  -- 激光雷达的数量。如果只有一个激光雷达，设置为 1
  num_laser_scans = 1,
  
  -- 多回波激光扫描的数量。如果没有多回波激光，设置为 0
  num_multi_echo_laser_scans = 0,
  
  -- 每次激光扫描的子扫描数，用于提高 SLAM 的性能。通常设置为 1
  num_subdivisions_per_laser_scan = 1,
  
  -- 点云传感器的数量。如果不使用点云传感器，设置为 0
  num_point_clouds = 0,
  
  -- 查找变换的超时时间，定义了在处理传感器数据时等待 TF 变换的最大时间
  lookup_transform_timeout_sec = 0.2,
  
  -- 定义子图发布的周期（秒），即地图的子图更新频率
  submap_publish_period_sec = 0.3,
  
  -- 定义位姿发布的周期（秒），即机器人的位姿更新频率
  pose_publish_period_sec = 5e-3,  -- 5 毫秒
  
  -- 定义轨迹发布的周期（秒），即机器人的轨迹更新频率
  trajectory_publish_period_sec = 30e-3,  -- 30 毫秒
  
  -- 激光雷达的采样比例。如果要处理所有的激光数据，设置为 1
  rangefinder_sampling_ratio = 1.,
  
  -- 里程计的采样比例。如果要处理所有的里程计数据，设置为 1
  odometry_sampling_ratio = 1.,
  
  -- 固定框架的位姿采样比例。对于固定框架的位姿采样，设置为 1
  fixed_frame_pose_sampling_ratio = 1.,
  
  -- IMU 的采样比例。如果没有 IMU，设置为 0。如果使用所有的 IMU 数据，设置为 1
  imu_sampling_ratio = 1.,
  
  -- 地标采样比例。如果没有使用地标，设置为 0。如果要处理所有的地标数据，设置为 1
  landmarks_sampling_ratio = 1.,
}

-- 配置 2D 地图构建。因为你在使用 2D SLAM，设置为 true
MAP_BUILDER.use_trajectory_builder_2d = true

-- 以下是 2D 轨迹构建器的具体参数设置

-- 每个子图包含的激光雷达数据帧数，较高的值可以使子图包含更多信息
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 激光雷达的最小有效距离（米），小于此值的激光数据将被忽略
TRAJECTORY_BUILDER_2D.min_range = 0.2

-- 激光雷达的最大有效距离（米），大于此值的激光数据将被忽略
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- 根据 RPLiDAR S2 的最大范围设置为 30 米

-- 用于填补缺失数据的光线长度。当激光雷达无法获取数据时，将假定该光线长度
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

-- 是否使用 IMU 数据。如果你的系统没有 IMU，设置为 false
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 是否在线使用相关扫描匹配，用于实时提高扫描匹配的精度
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 实时相关扫描匹配的线性搜索窗口（米），较大值增加匹配时间，但可以应对较大位移
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

-- 扫描匹配中位移误差的代价权重。值越高，系统对位移误差的敏感度越高
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.

-- 扫描匹配中旋转误差的代价权重。值越高，系统对旋转误差的敏感度越高
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 以下是位姿图优化的设置

-- Huber 损失函数的缩放参数，较大的值允许更多的误差不被严重惩罚
POSE_GRAPH.optimization_problem.huber_scale = 1e2

-- 每处理 35 个节点后进行一次位姿图优化
POSE_GRAPH.optimize_every_n_nodes = 35

-- 设置约束生成时的最小分数，较低分数的约束将被丢弃
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 全局定位时的最小分数，用于确定机器人全局位置
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 返回配置的选项
return options
