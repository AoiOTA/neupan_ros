# Livox Mid-360S 建图(gmapping)操作手册

> 适用对象:Scout Mini + Livox Mid-360S,ROS1 Noetic。
> 配套 `neupan_ros/mapping/` 目录,**自包含**:一条命令拉起 底盘 + 雷达 + TF + scan转换 + gmapping + RViz。
> 不依赖那 6 个迁移文件是否已落地,可在标定完成后直接独立测试建图。

---

## 0. 这个目录里有什么

| 文件 | 作用 |
|------|------|
| `mapping_test.launch` | 建图主启动:底盘 + Livox驱动 + 标定TF + 点云转scan + gmapping + RViz |
| `mapping.rviz` | 建图 RViz 配置(地图 + 2D scan + 车模型 + TF,Fixed Frame=map) |
| `Mapping_Manual.md` | 本手册 |

> 全是 launch/rviz/md,**不需要 catkin_make**,改完直接 `roslaunch`。

---

## 1. 数据流(先看懂)

```
底盘CAN ──odom→base_footprint──┐
                               ├─► gmapping ──► /map (+ map→odom)
Livox ─/livox/lidar(3D)─► p2l ─/scan(2D,base_link)─┘
          ▲                                   ▲
   base→livox 标定外参              footprint→base(z=0.181)
```

gmapping 需要两路输入:
1. **里程计 TF**:`odom→base_footprint`(底盘 CAN 提供)→ `base_footprint→base_link`(本 launch 静态 TF)
2. **2D 激光 `/scan`**:Livox 3D 点云投影到 base_link 水平面

gmapping 输出 `/map` 栅格地图,并发布 `map→odom` 补全 TF 树。

---

## 2. 前置条件(开建图前必须满足)

1. **标定已完成,外参已填入** —— 这是建图质量的前提。
   用 `calib_lidar.launch` 标好后,把 `lidar_calib_tf` 终端打印的那串
   `args="x y z yaw pitch roll base_link livox_frame"`,
   抄进 `mapping_test.launch` 第 3.2 节 `base_to_livox` 那一行(标了 ★★★)。
   > 没标定就先用文件里的 SolidWorks 初值(x=0.09563 z=0.355)也能跑,但点云贴合度差、地图会偏。

2. **网卡在雷达网段**:
   ```bash
   nmcli connection up "Lidar-Static"
   ping -c3 192.168.1.182        # 通
   ```

3. **底盘 CAN 已通**:
   ```bash
   candump can0                  # 能看到数据 (ID 211/221/241... 跳动)
   ```
   > 没接底盘 / 想用 rosbag 回放测试,见 §6「不带底盘」。

---

## 3. 启动建图

```bash
cd ~/neupan_ws
source devel/setup.bash
roslaunch neupan_ros mapping_test.launch
```

**开关**:

| 开关 | 效果 |
|------|------|
| `enable_base:=false`  | 不启动底盘(无 CAN 时,配合 rosbag 回放 `/livox/lidar` + odom) |
| `enable_model:=false` | 不显示车模型。**仅 `enable_base:=false` 时有意义**;带底盘时车模型由底盘 bringup 自动提供(强行再开会节点重名,本 launch 已做隔离) |
| `enable_rviz:=false`  | 不自动开 RViz |
| `min_height:=<m>` | `/scan` 切片下沿(默认 **0.0**)。**也算标定的一部分**,详见 [calibration/Calibration_Manual.md](../calibration/Calibration_Manual.md) §5.5 |
| `max_height:=<m>` | `/scan` 切片上沿(默认 1.0) |
| `range_min:=<m>` | 水平近距裁剪(默认 0.3,≈ Scout 半宽) |

> ⭐ `min_height`/`range_min` 是**交接物**:在 `calib_lidar.launch` 标定时定下的值,要和本 launch、`navigation.launch` **保持同一组**(默认已统一为 0.0 / 1.0 / 0.3)。

---

## 4. 建图操作流程

1. 启动后 RViz(默认俯视图)里应能看到:
   - 红色 `/scan` 轮廓(周围墙体形状)
   - 车模型在中心
   - 随着移动,灰白色 `/map` 栅格逐渐铺开(黑=障碍,白=空闲,灰=未知)

2. **手推 / 遥控小车慢速移动**建图,要点:
   - **慢**:平移 < 0.3 m/s,转弯尤其慢(滑移转向里程计转弯最不准)。
   - **多直线、少急转**:直走时地图最稳。
   - **回环**:绕一圈回到起点(loop closure),让 gmapping 闭合误差,地图更准。
   - 边走边看 RViz:墙线应保持单层、不重影。

3. **如果转弯出现地图重影 / 错层**:
   说明里程计旋转误差超出噪声模型。先停下,加大 `mapping_test.launch` 里 gmapping 的
   `srr`(0.5→0.7→1.0)和 `str`(0.3→0.5),重启再建。

---

## 5. 保存地图

地图建满意后,**另开一个终端**:

```bash
source ~/neupan_ws/devel/setup.bash
# 存到指定路径 (会生成 mymap.pgm + mymap.yaml 两个文件)
rosrun map_server map_saver -f ~/neupan_ws/src/neupan_ros/maps/mymap
```

> 建议存到你导航配置读取地图的目录(后续 AMCL/导航用 `map_server` 加载 `.yaml`)。
> 存图时**不要关 gmapping**,map_saver 抓的是当前 `/map`。存完再 Ctrl-C 全部关闭。

---

## 6. 不带底盘(rosbag 回放)测试

想先验证「雷达→scan→gmapping」链路而不动底盘:

```bash
# 终端A: 只起雷达+TF+scan+gmapping+rviz, 不起底盘
roslaunch neupan_ros mapping_test.launch enable_base:=false

# 终端B: 回放预先录的包 (需含 /livox/lidar 和 odom→base_footprint 的 tf)
rosbag play your_data.bag --clock
```

> 注意:gmapping 仍需要 `odom→base_footprint` 的 TF。若 bag 里没有里程计,gmapping 不会更新地图。

---

## 7. 验收标准

- `/scan` 的 `frame_id` 为 `base_link`:`rostopic echo /scan -n1 | grep frame_id`
- `/map` 持续刷新:`rostopic hz /map`
- TF 树完整无断:`rosrun tf view_frames` → `map→odom→base_footprint→base_link→livox_frame`
- 直线走廊在地图里是直的、墙是单层不重影;回环后起点能对上。

---

## 8. 常见问题排查

| 现象 | 原因 / 处理 |
|------|------------|
| RViz 没有 `/scan` | ① ping 不通雷达 / 驱动没起;② `rostopic echo /livox/lidar -n1` 看 3D 点云有没有;③ 确认驱动 `xfer_format=0` |
| 有 `/scan` 但 `/map` 不更新 | gmapping 缺里程计 TF。`rosrun tf tf_echo odom base_footprint` 应有输出;没有就是底盘没起/CAN 没通 |
| 地图转弯重影 / 错层 | 滑移转向里程计旋转误差大 → 加大 gmapping `srr`/`str`;并放慢转弯速度 |
| 地图整体扭曲 / 漂移 | 标定外参不准 → 回 `calib_lidar.launch` 重标,把新 args 填回本 launch |
| `scan` 里有地面/车体杂点 | 调 `pointcloud_to_laserscan` 的 `min_height`(增大滤地面)/ `max_height` |
| TF 报 extrapolation/警告 | 多半同时跑了别的发同名 TF 的 launch;建图只跑本 launch,先关掉标定/部署 launch |
| `base_to_livox` 没改 | 还在用 SolidWorks 初值,精度有限;标定后务必把 args 抄进第 3.2 节 |

---

## 9. 和标定 / 导航的关系

- **上游(标定)**:本 launch 第 3.2 节的 `base_to_livox` 外参来自 `calib_lidar.launch` 的标定结果。两者用的是同一串 args(顺序 `x y z yaw pitch roll`),抄过来即可。
- **下游(导航)**:建好的地图(`.pgm`+`.yaml`)供后续 AMCL 定位 + 导航使用。等你确认建图 OK,再落地那 6 个迁移文件,用 `deploy_scout.launch` / `test_navigation.launch` 跑完整导航 + NeuPAN。

> 注意:本 launch 是**独立测试用**,把外参、驱动、scan 转换都内联在一个文件里,方便单独验证建图。它与正式导航栈(`scout_tf.launch` + `deploy_scout.launch`)是两套并行入口,互不影响。
