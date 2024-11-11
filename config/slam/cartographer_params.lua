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
                                -- 使用IMU时，设置为 imu_link，否则，设置为 base_link
    use_odometry = true,  -- 是否使用里程计数据
    published_frame = "base_link",  -- 发布机器人的frame，用于位置发布
    odom_frame = "odom",  -- 里程计的frame名称
    provide_odom_frame = true,  -- 是否提供里程计frame
    publish_frame_projected_to_2d = true,  -- 是否将frame发布到2D平面
    use_odometry = true,  -- 是否使用里程计数据
    use_nav_sat = false,  -- 是否使用GPS数据
    use_landmarks = false,  -- 是否使用地标数据
    num_laser_scans = 1,  -- 使用的激光扫描数（此处配置一个RPLiDAR S2）
    num_multi_echo_laser_scans = 0,  -- 使用的多回波激光扫描数（RPLiDAR S2不支持）
    num_subdivisions_per_laser_scan = 1,  -- 每个激光扫描的子扫描数（提高处理速度）
    num_point_clouds = 0,  -- 使用的点云数据数（不使用3D点云）

    lookup_transform_timeout_sec = 0.3,  -- 查询tf变换的超时时间
    submap_publish_period_sec = 0.3,  -- 子图发布周期（提高地图更新频率）
    pose_publish_period_sec = 0.05,  -- 机器人位姿发布周期（实时性需求较高）
    trajectory_publish_period_sec = 0.05,  -- 轨迹发布周期（优化实时性和网络带宽）

    -- 传感器数据采样率配置
    rangefinder_sampling_ratio = 0.8,  -- 激光雷达数据采样率，1.0表示使用所有数据
    odometry_sampling_ratio = 1.0,     -- 里程计数据采样率
    fixed_frame_pose_sampling_ratio = 1.0,  -- 固定帧位姿采样率
    imu_sampling_ratio = 1.0,          -- IMU数据采样率
    landmarks_sampling_ratio = 1.0      -- 地标数据采样率
}

-- 2D SLAM配置
MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D SLAM（适合工厂环境）
MAP_BUILDER.num_background_threads = 4  -- 设置后台线程数；Orin Nano有8核，此处分配4核用于后台处理
                    -- 若负载测试表明处理器有富余性能且地图更新速度较慢，可以尝试提高至 5 或 6 核以提高实时性。

-- 设置RPLiDAR S2的测距范围
TRAJECTORY_BUILDER_2D.min_range = 0.1   -- 最小检测距离0.05米，适当增大 min_range（如 0.1~0.3）可以过滤掉干扰数据
TRAJECTORY_BUILDER_2D.max_range = 25.0  -- 最大检测距离30米（优化扫描大空间）

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 在数据缺失的情况下假设的射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 使用IMU数据提高精度
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10  -- IMU重力时间常数，影响IMU数据的平滑程度
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配（适合动态环境）
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- 用于扫描数据的体素滤波器大小，降低数据量
TRAJECTORY_BUILDER_2D.num_odometry_states = 1

-- 在线扫描匹配参数，用于提高位姿估计精度
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05  -- 平移搜索窗口，单位为米
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)  -- 旋转搜索窗口，单位为弧度（提高旋转匹配精度）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0  -- 平移代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- 旋转代价权重

-- Ceres优化器配置，用于精确的扫描匹配
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0  -- 占据空间的权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  -- 平移优化的权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30  -- 旋转优化的权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false  -- 是否使用非单调步长
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12  -- 最大迭代次数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1  -- 优化使用的线程数

-- 运动过滤器配置，用于过滤不必要的扫描
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.15  -- 两次扫描之间的最大时间间隔
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- 两次扫描之间的最大距离
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)  -- 两次扫描之间的最大角度

-- 为工厂/仓库环境优化的参数
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 体素过滤器的最大长度，较小体素以保留细节
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100  -- 保留的最小点数，避免过度滤波
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 25.0  -- 体素过滤器的最大作用范围

-- 回环检测的体素过滤器配置
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9  -- 回环检测时使用的体素最大长度
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100  -- 回环检测时的最小点数
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 25.0  -- 回环检测的最大范围

-- 子图配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40  -- 子图中包含的扫描数（适合大空间以获得稳定地图）
                                -- 如果发现生成的地图细节较多、重叠部分较复杂，可以适当减少此值（如 70），以增加子图生成速度。
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 子图栅格的分辨率，5厘米以平衡精度和性能
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"  -- 使用概率栅格地图
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"  -- 使用概率栅格数据插入器
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55  -- 击中栅格的概率
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49  -- 未击中栅格的概率

-- 回环检测参数
POSE_GRAPH.optimize_every_n_nodes = 40  -- 每90个节点执行一次图优化，与子图的num_range_data一致
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 用于约束构建的扫描数据采样率
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 最大约束距离，限制回环检测的范围
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 回环检测的最小匹配得分，值越高回环检测越严格
                                -- 如果实际测试中回环检测成功率较低，可以将 min_score 调低至 0.6，以增加回环检测的容忍度。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 全局定位的最小匹配得分，用于提高重定位准确性
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- 回环检测中平移部分的权重
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- 回环检测中旋转部分的权重

-- 位姿图优化器配置
POSE_GRAPH.optimization_problem = {
    huber_scale = 1e2,  -- Huber损失函数的缩放因子，用于鲁棒优化
    acceleration_weight = 1.0,  -- 加速度权重，影响轨迹的平滑程度
    rotation_weight = 1.0,  -- 旋转权重，影响位姿图中的旋转校正
    local_slam_pose_translation_weight = 1e5,  -- 局部SLAM中平移权重
    local_slam_pose_rotation_weight = 1e5,  -- 局部SLAM中旋转权重
    odometry_translation_weight = 1e5,  -- 里程计平移权重
    odometry_rotation_weight = 1e4,  -- 里程计旋转权重
    fixed_frame_pose_translation_weight = 1e1,  -- 固定帧位姿平移权重
    fixed_frame_pose_rotation_weight = 1e2,  -- 固定帧位姿旋转权重
    fixed_frame_pose_use_tolerant_loss = true,  -- 是否对固定帧位姿使用容忍损失函数
    fixed_frame_pose_tolerant_loss_param_a = 1,  -- 容忍损失函数参数a
    fixed_frame_pose_tolerant_loss_param_b = 1,  -- 容忍损失函数参数b
    use_online_imu_extrinsics_in_3d = false,  -- 是否在3D中使用在线IMU外参
    fix_z_in_3d = true,  -- 是否在3D中固定Z轴，2D SLAM中通常设为true
    log_solver_summary = false,  -- 是否记录求解器的摘要信息
    ceres_solver_options = {  -- Ceres求解器的配置选项
        use_nonmonotonic_steps = false,  -- 是否使用非单调步长
        max_num_iterations = 15,  -- 最大迭代次数
        num_threads = 2,  -- 优化使用的线程数
    }
}

return options  -- 返回配置
