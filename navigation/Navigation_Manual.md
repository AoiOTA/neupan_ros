# Livox Mid-360S 自主导航(AMCL + A* + NeuPAN)操作手册

> 适用对象:Scout Mini + Livox Mid-360S,ROS1 Noetic。
> 配套 `neupan_ros/navigation/` 目录,**自包含**:一条命令拉起 底盘 + 雷达 + TF + scan + 地图 + 定位 + 全局规划 + NeuPAN + RViz。
> 导航天然包含定位 —— 本目录用一个 launch + `localization_only` 开关,把"先验证定位、再放开自主导航"做成两步。

---

## 0. 这个目录里有什么

| 文件 | 作用 |
|------|------|
| `navigation.launch` | 导航主启动;`localization_only:=true` 只定位不动,默认完整自主导航 |
| `navigation.rviz` | 导航 RViz(地图 + 代价地图 + 粒子云 + scan + 全局路径 + NeuPAN 参考/轨迹/障碍标记 + 车模型) |
| `Navigation_Manual.md` | 本手册 |

> 全是 launch/rviz/md,**不需要 catkin_make**。

---

## 1. 数据流 / 架构

```
            RViz "2D Pose Estimate" ──► /initialpose ──► AMCL
            RViz "2D Nav Goal" ──► /move_base_simple/goal ──► global_planner(A*)
                                                                  │ /global_planner/plan (绿线)
                                                                  ▼
地图 /map ──► AMCL ──(map→odom)──►   NeuPAN(MPC/DUNE) ──► /cmd_vel ──► 底盘
/scan(base_link) ──► AMCL + NeuPAN ──┘     │ /neupan_plan(红) /neupan_initial_path(蓝) /障碍marker
```

- **定位**:`map_server` 载入地图,`AMCL` 用 `/scan` 在地图里定位,发布 `map→odom`。
- **全局规划**:`global_planner`(A*)根据目标点在地图上算一条全局路径(绿线)。
- **局部规划/控制**:`NeuPAN` 跟踪全局路径,同时用 `/scan` 实时避障,直接输出 `/cmd_vel` 驱动底盘。

---

## 2. 前置条件

1. **标定外参已填入**:把 `calib_lidar.launch` 标定得到的 `args` 抄进 `navigation.launch` 第 3.2 节 `base_to_livox`(与 `mapping_test.launch` 用同一串)。
2. **地图已就绪**:用 `mapping_test.launch` 建好图并保存,然后通过 `map_file:=` 指定:
   ```bash
   roslaunch neupan_ros navigation.launch map_file:=/home/lyb/neupan_ws/src/neupan_ros/maps/mymap.yaml
   ```
   > 默认 `map_file=~/maps/room/room.yaml`(旧图)。**用你自己新建的 Mid-360S 地图最准**。
3. **网络/CAN 已通**:`nmcli connection up "Lidar-Static"` + `ping 192.168.1.182`;`candump can0` 有数据。
4. **DUNE 模型存在**:`config/scout/neupan_planner_scout.yaml` 与 `model/custom/scout_dune.pth`(已确认在仓库内)。

---

## 3. ⚠️ 安全须知(完整导航会真实驱动小车,务必先读)

- **首次完整导航前,先把小车架空(轮子离地)**,或放在足够空旷、无人无障碍处。
- **手边常备急停**:Scout Mini 物理急停按钮 / 手柄;或随时 `Ctrl-C` 关闭 launch(会停发 `/cmd_vel`)。
- **限速起步**:首次把 NeuPAN 速度上限调小(见 §8),确认行为正常再放开。
- **先定位、后导航**:务必先用 `localization_only:=true` 确认 AMCL 定位准、`/scan` 与地图贴合,**再**放开自主导航。定位不准就让车自己跑 = 撞墙。
- **下发目标前看清全局路径**:RViz 里绿线(全局路径)应绕开障碍且合理,再让它跑。
- 启动 launch **本身不会让车动** —— NeuPAN 要等你用 "2D Nav Goal" 下发目标后才输出 `/cmd_vel`。

---

## 4. 第一步:纯定位验证(不动,安全)

```bash
cd ~/neupan_ws && source devel/setup.bash
roslaunch neupan_ros navigation.launch localization_only:=true \
    map_file:=/path/to/your_map.yaml
```

此模式只起 底盘 + 雷达 + TF + scan + map_server + AMCL + RViz,**不起 global_planner / NeuPAN,不发 /cmd_vel**(小车不会动)。

> ⚠️ **"纯定位"仍需雷达(和底盘)物理在线**:Livox 驱动与底盘驱动节点是 `required=true`,一旦雷达没连上 / CAN 断了,该节点退出会**触发 roslaunch 终止整个 launch**(连 AMCL/RViz 一起关),而不是降级成"只定位"。
> - 无底盘的台架,可加 `enable_base:=false` 跳过底盘(此时需自己提供 `odom→base_footprint`,如 rosbag);
> - 但**雷达 include 没有对应开关**,没接雷达时本模式无法空跑。纯验证 TF/配置请用 rosbag 回放 `/livox/lidar`。

验证步骤:
1. RViz 里应看到:地图、红色粒子云(`/particlecloud`)、绿色 `/scan`、车模型。
2. 点工具栏 **"2D Pose Estimate"**,在地图上车的真实位置点击并拖出朝向 → 给 AMCL 初始位姿。
3. **手推小车慢走几米 + 转弯**:粒子云应快速**收敛成一小簇**,且 `/scan` 红绿线与地图墙体**贴合**。
4. 收敛好、scan 贴墙 = 定位 OK,可以进入第二步。否则见 §7 排错(别带着发散的定位去自主导航)。

---

## 5. 第二步:完整自主导航(会动)

> 先确认 §3 安全须知与 §4 定位 OK。

```bash
roslaunch neupan_ros navigation.launch map_file:=/path/to/your_map.yaml
```

操作:
1. 同 §4 先用 **"2D Pose Estimate"** 把定位摆正(粒子云收敛)。
2. 点 **"2D Nav Goal"**,在地图空闲处点一个目标 + 拖朝向。
3. 应看到:**绿色全局路径**(A* 输出)→ **蓝色 NeuPAN 参考线** + **红色 NeuPAN 预测轨迹** + 障碍 marker。
4. NeuPAN 输出 `/cmd_vel`,小车开始沿路径走并实时避障,到点附近停下。
5. 中途要停:`Ctrl-C` 或物理急停。

---

## 6. 验收标准

- `/scan` 的 `frame_id` = `base_link`:`rostopic echo /scan -n1 | grep frame_id`
- TF 完整:`rosrun tf view_frames` → `map→odom→base_footprint→base_link→livox_frame`
- 定位:推车时粒子云收敛、scan 贴合地图墙体。
- 规划:下发目标后 `/global_planner/plan` 有路径(`rostopic echo -n1`)。
- 控制:`rostopic echo /cmd_vel` 在导航时有非零输出;到点后归零。
- 避障:在路径上临时放障碍,NeuPAN 红色预测轨迹应绕开。

---

## 7. 常见问题排查

| 现象 | 原因 / 处理 |
|------|------------|
| 粒子云不收敛 / 定位发散 | ① 标定外参没填准 → 重标;② `/scan` 与地图不匹配(地图是旧雷达建的)→ 用新图;③ 转弯发散是滑移转向特性,amcl.yaml 已调高 `odom_alpha`,仍发散就再加大 |
| `/scan` 与地图朝向/尺度对不上 | 地图分辨率/坐标问题,或定位初值给错 → 重给 2D Pose Estimate |
| 下发目标后无绿色全局路径 | ① 目标点落在障碍/未知区 → 换空闲处;② 定位没摆正,机器人在 costmap 外;③ 看 global_planner 终端报错 |
| 有全局路径但小车不动 | ① `rostopic echo /cmd_vel` 有没有输出;② NeuPAN 终端是否报错(模型/配置);③ 底盘是否使能(CAN、急停是否按下) |
| NeuPAN 启动报错 | 检查 `dune_model` 路径(`model/custom/scout_dune.pth`)与 `neupan_config` 是否存在;Python 依赖(neupan)是否装好 |
| 小车乱撞 / 避障迟钝 | 定位不准是首因;其次 NeuPAN 收缩/速度参数(neupan_planner_scout.yaml)需按车调 |
| TF extrapolation 警告 | 多半同时跑了别的发同名 TF 的 launch;导航只跑本 launch,先关掉 calib/mapping |
| `cmd_vel` 有输出但车不走 | 底盘急停/未上电/CAN 中断;`candump can0` 确认底盘在线 |

---

## 8. 参数微调(实车按需)

| 参数 | 文件 | 说明 |
|------|------|------|
| NeuPAN 速度/加速度上限 | `config/scout/neupan_planner_scout.yaml` | 首测调小,稳了再放开 |
| `scan_downsample` (默认4) | `navigation.launch` | 点太密 CPU 高→加大;避障漏点→减小 |
| `scan_range` (默认0.4 10.0) | `navigation.launch` | NeuPAN 考虑的障碍距离窗口 |
| AMCL `odom_alpha1/4` | `config/scout/amcl.yaml` | 转弯定位发散→加大(滑移转向) |
| A* / costmap 膨胀 | `planner_a_star.yaml` / `costmap_global.yaml` | 路径贴墙太近→加大膨胀半径 |

---

## 9. 与标定 / 建图的关系 + 方案A 说明

- **外参**:`base_to_livox` 来自 `calib_lidar.launch` 标定结果,与 `mapping_test.launch` 同一串 args。
- **地图**:`map_file` 用 `mapping_test.launch` 建的图最准。
- **方案A(重要)**:NeuPAN 节点的 `lidar_frame` 这里**写死 `base_link`**,而不是 `livox_frame`。
  因为 `/scan` 已被 `pointcloud_to_laserscan` 投影到 `base_link` 系;若写 `livox_frame`,`neupan_core` 查 `map←livox_frame` 会**重复叠加一次安装外参**,障碍点整体朝前偏 ~9.6cm。写 `base_link` 才与 `/scan` 实际所在坐标系一致。
  > 这与你 docs 里《迁移方案》第 10 节"方案A"一致。`scout_tf.launch` 的外参不受影响(它服务于 scan 投影那一步)。

> 注意:本 launch 是**独立测试入口**(外参/驱动/scan 都内联),与正式 `deploy_scout.launch` 是两套并行入口。等你确认导航 OK,再按 docs 落地那 6 个迁移文件,让 `deploy_scout.launch` 也走 Mid-360S。
