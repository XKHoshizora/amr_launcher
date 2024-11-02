# AMR Launcher

## 简介
AMR Launcher 是统合了所有 AMR 自动导航所需工具的启动器包。

## 使用方法

### 1. SLAM 制图

运行以下命令来启动 SLAM 制图：

```bash
roslaunch amr_launcher amr_launch_slam.launch
```

SLAM 节点启动后，便可以开始手动控制 AMR 进行地图制作。地图制作完成后使用以下命令来保存地图文件：

```bash
roslaunch amr_launcher amr_map_saver.launch
```

地图文件会默认保存在 `amr_launcher/map` 目录下。

### 2. AMR 自动导航

运行以下命令来启动导航：

```bash
roslaunch amr_launcher amr_launch.launch
```

### 3. AMR 多点导航

由于在 `amr_launch.launch` 中已经配置了对多点导航的支持，因此可以直接使用 RViz 工具栏中显示的 `Add Waypoint` 工具来添加多个导航点。添加完成之后，可以运行以下命令保存导航点：

```bash
roslaunch amr_launcher amr_waypoint_saver.launch
```

导航点文件会默认保存在 `amr_launcher/waypoint` 目录下。

然后，运行以下命令来启动多点导航：

```bash
roslaunch amr_launcher amr_waypoint_go.launch
```

AMR 将按照添加的导航点依次完成导航。
