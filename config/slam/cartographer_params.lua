-- Cartographer Configuration for Factory/Warehouse Environment
-- Platform: Jetson Orin Nano
-- Sensors: RPLiDAR S2, IMU, Odometry

include "map_builder.lua"  -- 引入地图构建模块
include "trajectory_builder.lua"  -- 引入轨迹构建模块

options = {
    map_builder = MAP_BUILDER,  -- 设置地图构建器配置
    trajectory_builder = TRAJECTORY_BUILDER,  -- 设置轨迹构建器配置
    map_frame = "map",  -- 全局地图的frame名称
    tracking_frame = "imu_link",  -- 跟踪frame（IMU的frame），通常用于追踪机器人的运动
                                -- 使用IMU时，设置为 imu_link，否则为 base_link
    published_frame = "base_link",  -- 发布的frame名称
    odom_frame = "odom",  -- 设置 odom 的frame名称
    provide_odom_frame = true,  -- 是否提供 odom frame
    publish_frame_projected_to_2d = true,  -- 是否将发布的frame投影到2D平面
    use_odometry = true,  -- 是否使用里程计数据
    use_nav_sat = false,  -- 是否使用导航卫星数据（如GPS）
    use_landmarks = false,  -- 是否使用地标（如已知位置的物体）
    num_laser_scans = 1,  -- 激光雷达数量
    num_multi_echo_laser_scans = 0,  -- 多回波激光雷达数量
    num_subdivisions_per_laser_scan = 10,  -- 每次激光雷达扫描的细分数
    num_point_clouds = 0,  -- 点云数量
    lookup_transform_timeout_sec = 0.3,  -- 查找坐标变换的超时时间（秒）
    submap_publish_period_sec = 0.3,  -- 子图发布周期（秒）
    pose_publish_period_sec = 5e-3,  -- 位姿发布周期（秒）
    trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期（秒）
    rangefinder_sampling_ratio = 0.8,  -- 激光雷达采样比例
    odometry_sampling_ratio = 1.0,  -- 里程计采样比例
    fixed_frame_pose_sampling_ratio = 1.0,  -- 固定frame位姿采样比例
    imu_sampling_ratio = 1.0,  -- IMU采样比例
    landmarks_sampling_ratio = 1.0  -- 地标采样比例
    publish_to_tf = true,          -- 确保TF发布
    use_pose_extrapolator = true   -- 使用位姿外推
}

-- 2D SLAM基础配置
MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D轨迹构建器
MAP_BUILDER.num_background_threads = 4  -- 后台线程数

-- 激光雷达配置
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10  -- 累积的激光雷达数据数量
TRAJECTORY_BUILDER_2D.min_range = 0.05  -- 最小有效测距范围
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- 最大有效测距范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 缺失数据光线的长度
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- IMU重力时间常数
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- 是否使用在线扫描匹配
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- 体素滤波器尺寸

-- 扫描匹配配置
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  -- 平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- 旋转权重

-- 子图配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50  -- 每个子图的激光数据数量
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 子图的分辨率

-- 回环检测配置
POSE_GRAPH.optimize_every_n_nodes = 50  -- 每隔多少节点进行一次优化
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 约束构建的最小分数
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 最大约束距离

return options  -- 返回配置选项
