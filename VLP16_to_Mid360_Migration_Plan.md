# Velodyne VLP-16 → Livox Mid-360 迁移方案

> **日期**: 2026-06-21
> **状态**: 待执行
> **SN 码**: ARMCP5M0030282 → IP = 192.168.1.182

---

## 1. 架构变化

```
当前 TF 树:
  map → odom → base_footprint → base_link → velodyne
                         ↑                  ↑
                    z=0.181           x=0.15, z=0.554
                                      pitch=-0.068, roll=0.01

目标 TF 树:
  map → odom → base_footprint → base_link → livox_frame
                         ↑                  ↑
                    z=0.181           x=0.09563, z=0.355
                                      pitch=0, roll=0
```

| 参数 | 旧值 (VLP-16) | 新值 (Mid-360) | 说明 |
|------|---------------|----------------|------|
| X 偏移 | 0.15 m | 0.09563 m | SolidWorks 测量值 |
| Y 偏移 | 0 | 0 | 左右居中 |
| Z 偏移 | 0.554 m | 0.355 m | SolidWorks 测量值 |
| Pitch | -0.068 rad | 0 | 默认正装 |
| Roll | 0.010 rad | 0 | 默认水平 |
| frame_id | `velodyne` | `livox_frame` | |
| LiDAR IP | 192.168.1.201 | 192.168.1.182 | SN 后两位=82 |
| 驱动包 | `velodyne_pointcloud` | `livox_ros_driver2` | |

---

## 2. IP 配置说明

根据 Livox 使用手册，Mid-360 默认 IP 规则为：
> `192.168.1.1XX`，其中 XX = SN 码最后两位数字

本设备 SN 码为 **ARMCP5M0030282**，后两位为 **82**，因此：

| 项目 | IP |
|------|-----|
| Mid-360 LiDAR IP | **192.168.1.182** |
| 工控机 Host IP | **192.168.1.5** (需确认) |
| 子网掩码 | 255.255.255.0 |
| 默认网关 | 192.168.1.1 |

> ⚠️ 首次使用建议直连，不经过路由器。

---

## 3. 修改文件清单

| # | 文件 | 操作 |
|---|------|------|
| 1 | `livox_ros_driver2/config/MID360s_config.json` | ✏️ 修改 IP |
| 2 | `neupan_ros/launch/scout_tf.launch` | ✏️ 修改 TF 外参 |
| 3 | `neupan_ros/launch/deploy_scout.launch` | ✏️ 修改驱动 + scan 转换 |
| 4 | `neupan_ros/launch/mapping_scout.launch` | ✏️ 修改驱动 + scan 转换 |
| 5 | `neupan_ros/launch/test_navigation.launch` | ✏️ 修改驱动 + scan 转换 |
| 6 | `neupan_ros/launch/livox_to_scan.launch` | ➕ 新建 |

---

## 4. 详细修改内容

### 4.1 `livox_ros_driver2/config/MID360s_config.json`

```diff
- "ip": "192.168.1.12",
+ "ip": "192.168.1.182",
```

完整配置参考:

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "host_ip": "192.168.1.5",
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.182",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

> **注意**: `extrinsic_parameter` 保持全零。外参由 TF (`scout_tf.launch`) 统一管理，避免双重变换。

---

### 4.2 `neupan_ros/launch/scout_tf.launch`

```diff
- <!-- 1. Base to Velodyne -->
- <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_velodyne"
-       args="0.15 0 0.554 0 -0.068 0.01 base_link velodyne" />
+ <!-- 1. Base to Livox Mid-360 (SolidWorks 测量值，x=95.63mm, z=355mm) -->
+ <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_livox"
+       args="0.09563 0 0.355 0 0 0 base_link livox_frame" />
```

> `footprint_to_base` (z=0.181) **保持不变**。

**修改后完整文件:**

```xml
<?xml version="1.0"?>
<launch>
    <!--
    =======================================================
    Scout Mini 工业级精细标定 TF 参数
    雷达: Livox Mid-360
    外参来源: SolidWorks 测量 (X=95.63mm, Z=355mm, 其余默认0)
    =======================================================
    -->

    <!--
    1. Base to Livox Mid-360
    参数顺序: x y z yaw pitch roll parent_frame child_frame

    X = 0.09563 : SolidWorks 测量前向偏移
    Y = 0.0     : 左右居中
    Z = 0.355   : SolidWorks 测量高度偏移
    Yaw = 0     : 默认正装
    Pitch = 0   : 默认水平
    Roll  = 0   : 默认水平
    -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_livox"
          args="0.09563 0 0.355 0 0 0 base_link livox_frame" />

    <!--
    2. Footprint to Base (定地)
    修正 URDF 模型中轮轴中心与地面的物理高度差。

    Z = 0.181   : 实测白色轮子模型底部刚好相切于地面网格线
    -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="footprint_to_base"
          args="0 0 0.181 0 0 0 base_footprint base_link" />

</launch>
```

---

### 4.3 `neupan_ros/launch/deploy_scout.launch`

**改动 1 — 第26行，lidar_frame 默认值:**

```diff
- <arg name="lidar_frame" default="velodyne"/>
+ <arg name="lidar_frame" default="livox_frame"/>
```

**改动 2 — 第43-52行，驱动替换:**

```diff
- <!-- 1.2 Velodyne VLP-16 雷达 -->
- <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
-     <arg name="device_ip" value="192.168.1.201"/>
-     <arg name="frame_id" value="$(arg lidar_frame)"/>
-     <arg name="port" value="2368"/>
-     <arg name="min_range" value="0.3"/>
-     <arg name="max_range" value="100.0"/>
- </include>
+ <!-- 1.2 Livox Mid-360 雷达 -->
+ <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
+     <arg name="xfer_format" value="0"/>
+     <arg name="publish_freq" value="10.0"/>
+     <arg name="multi_topic" value="0"/>
+     <arg name="msg_frame_id" value="$(arg lidar_frame)"/>
+ </include>
```

**改动 3 — 第60-67行，scan 转换替换:**

```diff
- <include file="$(find neupan_ros)/launch/velodyne_to_scan.launch">
-     <arg name="manager" value="velodyne_nodelet_manager"/>
-     <arg name="target_frame" value="base_link"/>
- </include>
+ <include file="$(find neupan_ros)/launch/livox_to_scan.launch">
+     <arg name="target_frame" value="base_link"/>
+ </include>
```

---

### 4.4 `neupan_ros/launch/mapping_scout.launch`

**改动 1 — 第38-55行，驱动替换:**

```diff
- <!-- 1.2 Velodyne VLP-16 驱动 & Nodelet 管理器 -->
- <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
-     <arg name="device_ip" value="192.168.1.201"/>
-     <arg name="port" value="2368"/>
-     <arg name="frame_id" value="velodyne"/>
-     <arg name="min_range" value="0.3"/>
-     <arg name="max_range" value="100.0"/>
- </include>
+ <!-- 1.2 Livox Mid-360 雷达 -->
+ <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
+     <arg name="xfer_format" value="0"/>
+     <arg name="publish_freq" value="10.0"/>
+     <arg name="multi_topic" value="0"/>
+     <arg name="msg_frame_id" value="livox_frame"/>
+ </include>
```

**改动 2 — 第71-75行，scan 转换替换:**

```diff
- <include file="$(find neupan_ros)/launch/velodyne_to_scan.launch">
-     <arg name="manager" value="velodyne_nodelet_manager"/>
-     <arg name="target_frame" value="base_link"/>
- </include>
+ <include file="$(find neupan_ros)/launch/livox_to_scan.launch">
+     <arg name="target_frame" value="base_link"/>
+ </include>
```

---

### 4.5 `neupan_ros/launch/test_navigation.launch`

**改动 1 — 第15行，lidar_frame 默认值:**

```diff
- <arg name="lidar_frame" default="velodyne"/>
+ <arg name="lidar_frame" default="livox_frame"/>
```

**改动 2 — 第23-29行，驱动替换:**

```diff
- <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
-     <arg name="device_ip" value="192.168.1.201"/>
-     <arg name="frame_id" value="$(arg lidar_frame)"/>
-     <arg name="port" value="2368"/>
-     <arg name="min_range" value="0.3"/>
-     <arg name="max_range" value="100.0"/>
- </include>
+ <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
+     <arg name="xfer_format" value="0"/>
+     <arg name="publish_freq" value="10.0"/>
+     <arg name="multi_topic" value="0"/>
+     <arg name="msg_frame_id" value="$(arg lidar_frame)"/>
+ </include>
```

**改动 3 — 第33-36行，scan 转换替换:**

```diff
- <include file="$(find neupan_ros)/launch/velodyne_to_scan.launch">
-     <arg name="manager" value="velodyne_nodelet_manager"/>
-     <arg name="target_frame" value="base_link"/>
- </include>
+ <include file="$(find neupan_ros)/launch/livox_to_scan.launch">
+     <arg name="target_frame" value="base_link"/>
+ </include>
```

---

### 4.6 `neupan_ros/launch/livox_to_scan.launch` (新建)

```xml
<?xml version="1.0"?>
<launch>
    <!--
    Livox Mid-360 → /scan (Nodelet 高性能版)

    架构说明:
    Livox 驱动不使用 Velodyne 的 nodelet_manager，因此为
    pointcloud_to_laserscan 启动独立的 nodelet manager，保持零拷贝性能。

    功能:
    将 Livox Mid-360 的 3D 点云投影到 base_link 水平面，
    生成 2D 激光数据供 AMCL / Gmapping 使用。

    输入: /livox/lidar  (sensor_msgs/PointCloud2, frame_id=livox_frame)
    输出: /scan        (sensor_msgs/LaserScan, frame_id=base_link)
    -->

    <!-- 目标坐标系 (投影到车身水平面) -->
    <arg name="target_frame" default="base_link"/>

    <!-- 独立的 Nodelet 管理器 -->
    <node pkg="nodelet" type="nodelet" name="livox_scan_manager"
          args="manager" output="screen"/>

    <!-- 3D 点云 → 2D 激光扫描 -->
    <node pkg="nodelet" type="nodelet" name="pointcloud_to_laserscan"
          respawn="true"
          args="load pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet livox_scan_manager">

        <!-- Livox 驱动发布的话题 -->
        <remap from="cloud_in" to="/livox/lidar"/>
        <remap from="scan" to="/scan"/>

        <rosparam subst_value="true">
            # 目标坐标系 (投影到车身水平坐标系)
            target_frame: $(arg target_frame)

            # TF 变换容差
            transform_tolerance: 0.01

            # ---------------------------------------------------------
            # 高度过滤 (在 base_link 坐标系下)
            # ---------------------------------------------------------
            # base_link 位于车轮轴心高度。
            # 地面位于 base_link 下方约 0.18m (Z ≈ -0.18)。
            # Livox 安装高度 Z=0.355m (低于原 VLP-16 的 0.554m)。

            # 过滤掉地面和车身自身结构
            min_height: 0.15

            # 忽略天花板和高处悬挂物
            max_height: 1.0

            # ---------------------------------------------------------
            # 扫描参数
            # ---------------------------------------------------------
            angle_min: -3.14159265359   # -180°
            angle_max: 3.14159265359    # +180°

            angle_increment: 0.005      # 角分辨率

            scan_time: 0.1              # 10Hz
            range_min: 0.3
            range_max: 100.0

            # 杂项
            use_inf: true
            inf_epsilon: 1.0
            concurrency_level: 1
        </rosparam>

    </node>
</launch>
```

---

## 5. 启动前检查清单

| # | 检查项 | 命令/方法 | 预期结果 |
|---|--------|-----------|----------|
| 1 | Mid-360 能 ping 通 | `ping 192.168.1.182` | 有回复 |
| 2 | 工控机 IP 在 192.168.1.X 网段 | `ifconfig` 或 `ip addr` | 如不是则修改 `host_ip` |
| 3 | `xfer_format=0` | 确认 launch include 传参 | 必须输出 PointCloud2 |
| 4 | `/livox/lidar` 有数据 | `rostopic echo /livox/lidar -n1` | 有点云数据 |
| 5 | TF 树完整 | `rosrun tf view_frames` | livox_frame 连接到 base_link |
| 6 | `/scan` 有数据 | `rostopic echo /scan -n1` | 有 LaserScan 数据 |

---

## 6. 后续微调参数

以下参数基于理论值设定，实际运行后可能需要微调：

| 参数 | 当前值 | 所在文件 | 微调方向 |
|------|--------|----------|----------|
| `Z` 轴外参 | 0.355 | `scout_tf.launch` | 点云离地偏高→减小；偏低→增大 |
| `X` 轴外参 | 0.09563 | `scout_tf.launch` | 旋转时墙壁重影→调整 X |
| `pitch` | 0 | `scout_tf.launch` | 地面点云前倾/后倾→微调 pitch |
| `roll` | 0 | `scout_tf.launch` | 左右点云高低不平→微调 roll |
| `min_height` | 0.15 | `livox_to_scan.launch` | 2D scan 有地面残留→增大 |
| `max_height` | 1.0 | `livox_to_scan.launch` | 期望检测高处障碍→增大 |
| `scan_time` | 0.1 | `livox_to_scan.launch` | 与 Livox publish_freq 匹配 |

---

## 7. 不修改的文件

以下文件/目录不受影响，保持原样：

| 文件/目录 | 原因 |
|-----------|------|
| `velodyne/` | 保留不动，作为备选方案 |
| `scout_ros/` | 底盘驱动，不涉及 LiDAR |
| `ugv_sdk/` | 底盘 SDK，不涉及 LiDAR |
| `NeuPAN/` | 算法核心，不涉及 LiDAR |
| `neupan_ros/config/scout/amcl.yaml` | AMCL 订阅 /scan，接口未变 |
| `neupan_ros/config/scout/neupan_planner_scout.yaml` | NeuPAN 参数，接口未变 |
| `neupan_ros/config/scout/planner_a_star.yaml` | A* 规划器参数，不涉及 |
| `neupan_ros/config/scout/costmap_global.yaml` | Costmap 参数，不涉及 |
| `livox_ros_driver2/` | 驱动本身无需修改 (除 IP config) |
| `Livox-SDK2/` | SDK，无需修改 |
