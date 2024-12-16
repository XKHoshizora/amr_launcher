include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
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
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.0,  -- 禁用 IMU 数据
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  use_imu_data = false, -- 禁用 IMU 数据

  num_accumulated_range_data = 10,
  min_range = 0.1,
  max_range = 25.0,
  missing_data_ray_length = 25.0,
  use_online_correlative_scan_matching = true,

  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,
    angular_search_window = math.rad(5.0),
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.0,
    translation_weight = 10.0,
    rotation_weight = 40.0,
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.0),
  },

  submaps = {
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    num_range_data = 120,
  },
}

return options
