# Livox Mid-360S 外参标定操作手册

> 适用对象:Scout Mini + Livox Mid-360S,ROS1 Noetic。
> 本手册配套 `neupan_ros/calibration/` 目录下的标定工具,**只用于标定外参**,
> 不启动底盘 / CAN / 导航 / NeuPAN,可独立运行,不影响你现有的 deploy/mapping/test 流程。

---

## 0. 这个目录里有什么

| 文件 | 作用 |
|------|------|
| `calib_lidar.launch` | 标定主启动文件:雷达驱动 + 车模型 + 动态 TF + 滑条 + RViz |
| `lidar_calib_tf.py` | 动态 TF 节点:把滑条数值实时发布成 `base_link → livox_frame` 变换 |
| `LidarCalib.cfg` | dynamic_reconfigure 参数定义(6 个滑条:x/y/z/roll/pitch/yaw 及范围) |
| `calib.rviz` | RViz 配置:车模型 + 3D 点云 + 2D scan + 地面网格 + 预设视角 |
| `Calibration_Manual.md` | 本手册 |

> 这些文件已在 `CMakeLists.txt` / `package.xml` 中登记。改完 `LidarCalib.cfg` 后需要重新 `catkin_make`;
> 改 launch / rviz / py 脚本逻辑则**不用**重新编译,直接重启 launch 即可。

---

## 1. 标定原理(先看懂,再动手)

整条 TF 链:

```
map → odom → base_footprint → base_link → livox_frame → (点云)
                    │z=0.181固定         │← 这一段就是我们要标定的外参
                  (Scout 离地高)      (雷达相对车体)
```

- **本次标定的目标**:确定 `base_link → livox_frame` 的 6 个外参 `x y z roll pitch yaw`。
- `base_footprint → base_link` 的 `z=0.181`(车体离地高)是 Scout Mini 固定值,标定时作为地面基准,不动它。
- 标定结果最终要抄回 `scout_tf.launch` 里的静态 TF,让导航/建图时使用。

**初始值(SolidWorks 理想测量)**:`x=0.09563, y=0, z=0.355, roll=0, pitch=0, yaw=0`(单位 m / rad)。
实物安装会有偏差,本手册就是用来在这个初值附近微调,让点云和真实世界对齐。

---

## 2. 启动前准备

1. **雷达上电**,网线接好(参考供电/电路文档,雷达 24V→伽冬 DCDC→12V 供电)。
2. **网卡切到雷达网段**并确认连通:
   ```bash
   nmcli connection up "Lidar-Static"      # 主机切到 192.168.1.100 网段
   ping -c 3 192.168.1.182                  # 雷达 IP,能通再继续
   ```
3. **确认没有别的 Livox 驱动 / RViz 在跑**(比如之前调试用的 `rviz_MID360s.launch`),
   否则会抢占端口或叠加 TF。有就先 `Ctrl-C` 关掉。

---

## 3. 启动标定

```bash
cd ~/neupan_ws
source devel/setup.bash
roslaunch neupan_ros calib_lidar.launch
```

启动后会弹出 **3 样东西**:

1. **RViz** —— Scout Mini 车模型 + 雷达点云(按高度 Z 上色)+ 红色 2D scan + 地面网格(每格 0.5m)。
2. **rqt_reconfigure** —— 左侧列表点 **`lidar_calib_tf`**,右边出现 6 个可拖动滑条。
3. **终端** —— 实时打印当前外参值,以及可直接抄走的 `args` 字符串。

**常用开关**(按需加在命令后面):

| 开关 | 效果 |
|------|------|
| `enable_scan:=false` | 只看 3D 点云,不生成 `/scan`(标定外参其实只需要 3D 点云) |
| `enable_rviz:=false` | 不自动开 RViz(想用自己的 RViz 时) |
| `enable_rqt:=false`  | 不自动开 rqt_reconfigure |
| `lidar_frame:=xxx`   | 改雷达坐标系名(默认 `livox_frame`,一般不用改) |

---

## 4. 标定步骤(核心)

> 原则:**拖一个滑条 → 看 RViz 点云实时变化 → 满意了再调下一个**。不用重启。
> 滑条太粗调不准时,直接在滑条旁边的**输入框里打精确数值**。

### 步骤 A — 调 Z(高度):让地面点云贴在网格平面上

1. RViz 右下角 **Views** 面板 → 双击选 **`侧视图_看点云是否水平贴地`**(侧视角)。
2. 看地面的点云(一大片平面点)。它应该正好落在 **z=0 的网格平面**上。
3. 拖 **`z`** 滑条:
   - 地面点云整体在网格**上方**飘 → 把 z **调小**;
   - 陷到网格**下方** → 把 z **调大**;
   - 直到地面点云贴合网格平面。
   - 理论值约 `0.355`,微调即可。

### 步骤 B — 调 pitch / roll(俯仰/横滚):让地面点云不倾斜

还在侧视图:

- 地面点云**前高后低 / 前低后高**(像斜坡)→ 调 **`pitch`**,直到地面变平。
- 换个角度看,地面点云**左高右低 / 左低右高** → 调 **`roll`**,直到左右也平。
- 调完 pitch/roll 后,地面 Z 可能又有点偏,**回步骤 A 复核一下 z**。理想安装下 pitch/roll 应该都接近 0。

### 步骤 C — 调 X 和 yaw:推车原地旋转,消除"重影"

1. Views 面板 → 选 **`俯视图_看旋转重影`**(正上方俯视)。
2. 把车停在有清晰墙壁/直边的地方,**原地慢慢旋转一圈**(或缓慢前后平移)。
3. 观察墙线:
   - 如果旋转时墙壁出现**重影 / 双线 / 厚墙**,说明雷达旋转中心和车体旋转中心没对上 → 调 **`x`**(和必要时 `y`),让墙线在旋转时**重合成一条细线**。
   - 如果直线方向整体**偏转了一个角度**(车头方向和点云朝向不一致)→ 调 **`yaw`**。
4. `x` 理论值约 `0.09563`,`y` 理论值 `0`,`yaw` 理论值 `0`,在此附近微调。

> 小贴士:重影法对 x/y 很灵敏,是标定平移最有效的方法。墙线越细、旋转时越不动,说明外参越准。

---

## 5. 取结果并写回(关键一步)

调到满意后,看 **`lidar_calib_tf` 节点所在终端**,它会打印类似:

```
[标定值] x=0.09563 y=0.00000 z=0.35500 roll=0.00000 pitch=0.00000 yaw=0.00000
         scout_tf 写法 -> args="0.09563 0.00000 0.35500 0.00000 0.00000 0.00000 base_link livox_frame"
```

把这串 **`args="..."`** 整段抄进 `scout_tf.launch` 里对应的雷达静态 TF
(`base_link → livox_frame` 的那个 `static_transform_publisher`)。

> **顺序已经帮你排好了**:`static_transform_publisher` 的 args 顺序是
> **`x y z yaw pitch roll parent child`**(注意是 yaw-pitch-roll,不是 roll-pitch-yaw),
> 终端打印的就是这个顺序,**直接复制即可,不会搞错**。

写回后,正常用 `deploy_scout.launch` / `mapping_scout.launch` 跑导航/建图,即用上新外参。

---

## 6. 验收标准

- 侧视图:地面点云平贴 z=0 网格,无明显倾斜。
- 俯视图:原地旋转时墙线收成**一条细线**,无重影。
- 车模型的雷达安装位置和点云的发出位置在视觉上吻合。
- (可选)开 `/scan` 时,2D 红线轮廓和 3D 点云、和真实环境一致。

---

## 7. 常见问题排查

| 现象 | 原因 / 处理 |
|------|------------|
| RViz 里没有点云 | ① `ping 192.168.1.182` 不通 → 检查网卡 `Lidar-Static` / 供电;② 有别的驱动占着端口 → 先关掉再启动;③ RViz 里 `PointCloud2` 的 Topic 是否为 `/livox/lidar` |
| 报 "port already in use" / 驱动启动失败 | 之前的 Livox 驱动没关干净。`Ctrl-C` 全关,或 `pkill -f livox`,再重启 |
| RViz 没车模型 / 报 robot_description 错 | `scout_description` 包没编译或没 source;确认 `model_xacro` 路径存在 |
| rqt_reconfigure 里找不到滑条 | 左侧列表里要点 **`lidar_calib_tf`** 这个节点名,展开后才有 6 个滑条 |
| 改了 `LidarCalib.cfg` 没生效 | `.cfg` 改动需 **重新 `catkin_make`**,再重开 launch |
| TF 报 extrapolation / 警告 | 多半是同时跑了别的发 `base_link→livox_frame` 的节点;标定时只用本 launch |
| 拖滑条点云不动 | 确认 `lidar_calib_tf` 节点终端有打印 `[标定值]`;没打印说明节点没起来,看终端报错 |

---

## 8. 注意事项

- 本 launch 用 `base_footprint→base_link` 的 `z=0.181` 作为地面基准。如果车体离地高变了(换轮子/底盘),改 `calib_lidar.launch` 第 50 行那个静态 TF。
- 欧拉角约定:`lidar_calib_tf.py` 用 `quaternion_from_euler(roll,pitch,yaw)`,与 `static_transform_publisher` 的 `setRPY(roll,pitch,yaw)` **完全一致**,抄回去数值不变。
- 滑条范围:`x∈[-0.5,0.6]`、`y∈[-0.3,0.3]`、`z∈[0,0.7]`、`roll/pitch∈[±0.5]`、`yaw∈[±3.15]`(rad)。
  觉得范围不够或想改默认值,编辑 `LidarCalib.cfg` 后 `catkin_make`。
- 标定全程不动底盘、不发速度指令,推车靠手推即可,安全。
