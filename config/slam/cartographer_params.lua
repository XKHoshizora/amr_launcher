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

-- 引入 Cartographer 的构建器配置
include "map_builder.lua"
include "trajectory_builder.lua"

-- 配置选项
options = {
  -- 地图构建器
  map_builder = MAP_BUILDER,
  -- 轨迹构建器
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 全局坐标系名称，通常为 "map"
  map_frame = "map",
  -- 机器人主坐标系，通常为底盘坐标系 "base_link"
  tracking_frame = "base_link",
  -- 发布机器人的位姿坐标系
  published_frame = "base_link",
  -- 里程计坐标系
  odom_frame = "odom",
  -- 是否由 Cartographer 提供 odom_frame。您的机器人有外部里程计，应设为 false
  provide_odom_frame = false,
  -- 是否将机器人位姿发布到 2D 平面
  publish_frame_projected_to_2d = false,

  -- 使用姿态外推器来提高轨迹跟踪稳定性
  use_pose_extrapolator = true,
  -- 是否使用外部里程计信息
  use_odometry = true,
  -- 是否使用卫星导航数据（如 GPS）
  use_nav_sat = false,
  -- 是否使用地标信息
  use_landmarks = false,

  -- 激光雷达配置（使用多回波激光雷达）
  num_laser_scans = 0,                     -- 单回波激光雷达数量
  num_multi_echo_laser_scans = 1,          -- 多回波激光雷达数量
  num_subdivisions_per_laser_scan = 10,    -- 每帧激光分段数，适当增大以提高匹配精度
  num_point_clouds = 0,                    -- 点云数量（3D SLAM 用，2D 不使用）

  -- TF 坐标变换的超时时间，单位秒
  lookup_transform_timeout_sec = 0.2,

  -- 子图发布周期（秒）
  submap_publish_period_sec = 0.3,
  -- 位姿发布周期（秒）
  pose_publish_period_sec = 5e-3,
  -- 轨迹发布周期（秒）
  trajectory_publish_period_sec = 30e-3,

  -- 数据采样率，1.0 表示不下采样
  rangefinder_sampling_ratio = 1.0,        -- 激光雷达数据采样率
  odometry_sampling_ratio = 1.0,          -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.0,  -- 固定帧位姿采样率（未使用）
  imu_sampling_ratio = 0.0,               -- IMU 数据采样率（未使用）
  landmarks_sampling_ratio = 1.0,         -- 地标数据采样率（未使用）
}

-- 启用 2D 轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 轨迹构建器 2D 配置
TRAJECTORY_BUILDER_2D = {
  -- 激光帧累计数量，用于构建轨迹，提高匹配质量。根据您的环境建议设置为 10
  num_accumulated_range_data = 10,

  use_imu_data = false  -- 明确禁用 IMU 数据

  -- 激光雷达的有效测量范围，需根据 RPLiDAR S2 的性能调整
  min_range = 0.05,                         -- 最小测量距离（小于此值的测量将被忽略）
  max_range = 25.0,                        -- 最大测量距离（大于此值的测量将被忽略）
  missing_data_ray_length = 25.0,          -- 未命中射线的虚拟长度，用于更新自由空间

  -- 使用在线相关扫描匹配，适用于动态环境或大范围移动
  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,            -- 平移搜索窗口（单位：米）
    angular_search_window = math.rad(5.0),-- 角度搜索窗口（单位：弧度）
  },

  -- 局部优化的 Ceres 扫描匹配器参数
  ceres_scan_matcher = {
    occupied_space_weight = 1.0,           -- 占据空间一致性权重
    translation_weight = 10.0,             -- 平移约束权重
    rotation_weight = 40.0,                -- 旋转约束权重
  },

  -- 运动过滤器，用于减少无效帧，降低计算量
  motion_filter = {
    max_time_seconds = 0.5,                -- 时间间隔阈值（秒）
    max_distance_meters = 0.2,             -- 距离变化阈值（米）
    max_angle_radians = math.rad(1.0),     -- 角度变化阈值（弧度）
  },

  -- 子图分辨率和构建配置
  submaps = {
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",      -- 2D 概率栅格地图
      resolution = 0.05,                  -- 子图分辨率（米/格）
    },
    num_range_data = 120,                  -- 每个子图包含的激光帧数（较大范围环境适当增大此值）
  },
}

return options
