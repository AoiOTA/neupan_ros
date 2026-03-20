#!/usr/bin/env python

"""
waypoint_patrol.py — 多目标点连续巡航管家 (Multi-Waypoint Patrol Manager)

功能:
  1. RECORDING 模式: 通过 RViz "Publish Point" 批量录入巡航点
  2. 实时调用 global_planner make_plan 服务，缝合子路径并预览
  3. 终端按 ENTER 切入 PATROLLING 模式，全路径一次性下发给 NeuPAN
  4. 监听 /neupan_arrived 防抖判定到达后，自动重新下发，实现无限循环

话题/服务:
  订阅: /clicked_point (geometry_msgs/PointStamped)
        /neupan_arrived (std_msgs/Bool)
  发布: /initial_path (nav_msgs/Path)         - 给 NeuPAN 的执行路径
        /patrol_full_path (nav_msgs/Path)     - 预览用完整缝合路径
        /patrol_markers (visualization_msgs/MarkerArray) - 目标点标记
  服务: /global_planner/planner/make_plan (nav_msgs/GetPlan)
  TF:   map -> base_link (读取当前位姿)
"""

import rospy
import threading
import sys
import numpy as np
from math import atan2, sqrt

import tf
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# ====================== 状态常量 ======================
STATE_RECORDING = "RECORDING"
STATE_PATROLLING = "PATROLLING"


class WaypointPatrol:
    def __init__(self):
        rospy.init_node("waypoint_patrol", anonymous=False)

        # ==================== 参数 ====================
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.plan_service_name = rospy.get_param(
            "~plan_service", "/global_planner/planner/make_plan"
        )
        self.arrival_debounce_k = rospy.get_param("~arrival_debounce_k", 15)
        self.loop_patrol = rospy.get_param("~loop_patrol", True)
        self.plan_tolerance = rospy.get_param("~plan_tolerance", 0.1)

        # ==================== 状态 ====================
        self.state = STATE_RECORDING
        self.waypoints = []           # list of (x, y) tuples
        self.stitched_path = None     # nav_msgs/Path — 缝合后的超长路径
        self.lap_count = 0
        self.arrive_count = 0         # 防抖计数器

        # ==================== TF ====================
        self.tf_listener = tf.TransformListener()

        # ==================== 服务代理 ====================
        rospy.loginfo(
            "[Patrol] Waiting for plan service: {}".format(self.plan_service_name)
        )
        try:
            rospy.wait_for_service(self.plan_service_name, timeout=30.0)
            self.make_plan = rospy.ServiceProxy(self.plan_service_name, GetPlan)
            rospy.loginfo("[Patrol] Plan service connected!")
        except rospy.ROSException:
            rospy.logfatal(
                "[Patrol] TIMEOUT waiting for service {}. Is global_planner running?".format(
                    self.plan_service_name
                )
            )
            sys.exit(1)

        # ==================== Publisher ====================
        self.path_pub = rospy.Publisher("/initial_path", Path, queue_size=1, latch=True)
        self.preview_pub = rospy.Publisher(
            "/patrol_full_path", Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/patrol_markers", MarkerArray, queue_size=1, latch=True
        )

        # ==================== Subscriber ====================
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb)
        rospy.Subscriber("/neupan_arrived", Bool, self.arrived_cb)

        # ==================== 键盘监听线程 ====================
        self._enter_pressed = False
        self._kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._kb_thread.start()

        rospy.loginfo("=" * 60)
        rospy.loginfo("[Patrol] Node started. State: RECORDING")
        rospy.loginfo("[Patrol] Use RViz 'Publish Point' tool to add waypoints.")
        rospy.loginfo("[Patrol] Press ENTER in this terminal to start patrol.")
        rospy.loginfo("=" * 60)

    # ==================================================================
    #                      获取当前机器人位姿
    # ==================================================================
    def _get_robot_pose(self):
        """从 TF 获取机器人在 map 坐标系下的当前位姿，返回 PoseStamped"""
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0)
            )
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return pose
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logwarn("[Patrol] TF lookup failed: {}".format(e))
            return None

    # ==================================================================
    #                      A* 子路径规划
    # ==================================================================
    def _call_make_plan(self, start_pose, goal_x, goal_y):
        """
        调用 global_planner 的 make_plan 服务，返回 nav_msgs/Path。
        失败则返回 None。
        """
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0

        try:
            resp = self.make_plan(start=start_pose, goal=goal, tolerance=self.plan_tolerance)
            if resp.plan and len(resp.plan.poses) >= 2:
                rospy.loginfo(
                    "[Patrol] make_plan OK: {} poses".format(len(resp.plan.poses))
                )
                return resp.plan
            else:
                rospy.logwarn(
                    "[Patrol] make_plan returned empty/short path to ({:.2f}, {:.2f})".format(
                        goal_x, goal_y
                    )
                )
                return None
        except rospy.ServiceException as e:
            rospy.logerr("[Patrol] make_plan service call failed: {}".format(e))
            return None

    # ==================================================================
    #                      子路径硬缝合
    # ==================================================================
    def _stitch_paths(self, sub_paths):
        """
        将多段 nav_msgs/Path 硬缝合成一条超长 Path。
        关键：去除连接处的重复点（前一段的最后一个点 ≈ 后一段的第一个点）
        """
        stitched = Path()
        stitched.header.frame_id = self.map_frame
        stitched.header.stamp = rospy.Time.now()

        for idx, sub_path in enumerate(sub_paths):
            poses = sub_path.poses
            if idx == 0:
                # 第一段：完整保留
                stitched.poses.extend(poses)
            else:
                # 后续段：跳过第一个点（与前一段末尾重复）
                if len(poses) > 1:
                    stitched.poses.extend(poses[1:])
                elif len(poses) == 1:
                    # 只有一个点，检查是否与前一个真的重复
                    if len(stitched.poses) > 0:
                        last = stitched.poses[-1]
                        curr = poses[0]
                        dx = last.pose.position.x - curr.pose.position.x
                        dy = last.pose.position.y - curr.pose.position.y
                        if sqrt(dx * dx + dy * dy) > 0.05:
                            stitched.poses.extend(poses)

        rospy.loginfo(
            "[Patrol] Stitched {} sub-paths -> {} total poses".format(
                len(sub_paths), len(stitched.poses)
            )
        )
        return stitched

    # ==================================================================
    #                      完整路径规划与缝合
    # ==================================================================
    def _compute_full_patrol_path(self):
        """
        根据当前已录入的 waypoints，规划完整巡航路径：
        当前位置 -> WP1 -> WP2 -> ... -> WPN -> (如果 loop) -> WP1 起点
        """
        if len(self.waypoints) < 1:
            rospy.logwarn("[Patrol] No waypoints to plan!")
            return None

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            rospy.logerr("[Patrol] Cannot get robot pose for planning!")
            return None

        # 构建完整的目标序列
        targets = list(self.waypoints)  # [(x,y), (x,y), ...]
        if self.loop_patrol and len(targets) >= 2:
            # 回到起始位置（机器人当前位置），形成闭环
            targets.append(
                (robot_pose.pose.position.x, robot_pose.pose.position.y)
            )

        # 逐段规划
        sub_paths = []
        current_start = robot_pose

        for i, (tx, ty) in enumerate(targets):
            rospy.loginfo(
                "[Patrol] Planning segment {}/{}: ({:.2f},{:.2f}) -> ({:.2f},{:.2f})".format(
                    i + 1,
                    len(targets),
                    current_start.pose.position.x,
                    current_start.pose.position.y,
                    tx,
                    ty,
                )
            )

            sub_path = self._call_make_plan(current_start, tx, ty)
            if sub_path is None:
                rospy.logerr(
                    "[Patrol] FAILED to plan segment {}! Aborting full path.".format(
                        i + 1
                    )
                )
                return None

            sub_paths.append(sub_path)

            # 更新下一段的起点为当前段的终点
            current_start = sub_path.poses[-1]

        # 缝合
        stitched = self._stitch_paths(sub_paths)
        return stitched

    # ==================================================================
    #                      RViz Marker 生成
    # ==================================================================
    def _publish_markers(self):
        """发布带编号文本的球体 Marker，标记所有已录入的目标点"""
        ma = MarkerArray()

        # 先发一个 DELETEALL 清除旧的
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, (wx, wy) in enumerate(self.waypoints):
            # 球体
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "patrol_spheres"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wx
            sphere.pose.position.y = wy
            sphere.pose.position.z = 0.5
            sphere.pose.orientation = Quaternion(0, 0, 0, 1)
            sphere.scale.x = 0.35
            sphere.scale.y = 0.35
            sphere.scale.z = 0.35
            sphere.color = ColorRGBA(0.0, 0.5, 1.0, 0.9)  # 蓝色
            sphere.lifetime = rospy.Duration(0)
            ma.markers.append(sphere)

            # 文本编号
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = rospy.Time.now()
            text.ns = "patrol_labels"
            text.id = 1000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wx
            text.pose.position.y = wy
            text.pose.position.z = 1.0
            text.pose.orientation = Quaternion(0, 0, 0, 1)
            text.scale.z = 0.4
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 白色文字
            text.text = "WP{}".format(i + 1)
            text.lifetime = rospy.Duration(0)
            ma.markers.append(text)

        self.marker_pub.publish(ma)

    # ==================================================================
    #                      回调函数
    # ==================================================================
    def clicked_point_cb(self, msg):
        """RViz 'Publish Point' 工具回调"""
        if self.state != STATE_RECORDING:
            rospy.logwarn(
                "[Patrol] Ignoring clicked point — not in RECORDING state."
            )
            return

        wx = msg.point.x
        wy = msg.point.y
        self.waypoints.append((wx, wy))

        rospy.loginfo(
            "[Patrol] *** Waypoint {} recorded: ({:.3f}, {:.3f}) ***".format(
                len(self.waypoints), wx, wy
            )
        )

        # 发布目标点 Marker
        self._publish_markers()

        # 每次新增点后，重新计算并预览完整路径
        if len(self.waypoints) >= 1:
            rospy.loginfo("[Patrol] Re-computing full patrol path for preview...")
            full_path = self._compute_full_patrol_path()
            if full_path is not None and len(full_path.poses) >= 2:
                self.stitched_path = full_path
                self.preview_pub.publish(self.stitched_path)
                rospy.loginfo(
                    "[Patrol] Preview updated: {} total poses. "
                    "Check /patrol_full_path in RViz!".format(
                        len(self.stitched_path.poses)
                    )
                )
            else:
                rospy.logwarn("[Patrol] Path computation failed. Preview not updated.")

        rospy.loginfo(
            "[Patrol] Total waypoints: {}. Press ENTER to start patrol.".format(
                len(self.waypoints)
            )
        )

    def arrived_cb(self, msg):
        """监听 NeuPAN 的到达广播"""
        if self.state != STATE_PATROLLING:
            return

        if msg.data:
            self.arrive_count += 1
            if self.arrive_count >= self.arrival_debounce_k:
                self.lap_count += 1
                rospy.loginfo(
                    "[Patrol] ======== LAP {} COMPLETED! ========".format(
                        self.lap_count
                    )
                )
                self.arrive_count = 0

                if self.loop_patrol:
                    rospy.loginfo(
                        "[Patrol] Re-dispatching full path for lap {}...".format(
                            self.lap_count + 1
                        )
                    )
                    # 等待短暂时间让 NeuPAN 稳定
                    rospy.sleep(0.5)
                    self._dispatch_path()
                else:
                    rospy.loginfo("[Patrol] Single patrol complete. Stopping.")
                    self.state = STATE_RECORDING
        else:
            # 收到 False，重置防抖计数器
            self.arrive_count = 0

    # ==================================================================
    #                      路径下发
    # ==================================================================
    def _dispatch_path(self):
        """将缝合后的超长路径一次性下发给 NeuPAN"""
        if self.stitched_path is None or len(self.stitched_path.poses) < 2:
            rospy.logerr("[Patrol] No valid stitched path to dispatch!")
            return

        # 更新时间戳
        self.stitched_path.header.stamp = rospy.Time.now()
        for pose in self.stitched_path.poses:
            pose.header.stamp = rospy.Time.now()

        self.path_pub.publish(self.stitched_path)
        rospy.loginfo(
            "[Patrol] >>> PATH DISPATCHED to /initial_path ({} poses) <<<".format(
                len(self.stitched_path.poses)
            )
        )

    # ==================================================================
    #                      键盘监听 (后台线程)
    # ==================================================================
    def _keyboard_listener(self):
        """
        后台线程：阻塞等待用户在终端按 ENTER。
        按下后设置标志位，由主循环处理状态转换。
        """
        while not rospy.is_shutdown():
            try:
                if sys.version_info[0] >= 3:
                    input()  # Python 3
                else:
                    raw_input()  # Python 2
                self._enter_pressed = True
            except EOFError:
                rospy.sleep(1.0)

    # ==================================================================
    #                      主循环
    # ==================================================================
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz 足够

        while not rospy.is_shutdown():
            # 检测 ENTER 键
            if self._enter_pressed:
                self._enter_pressed = False

                if self.state == STATE_RECORDING:
                    if self.stitched_path is None or len(self.stitched_path.poses) < 2:
                        rospy.logwarn(
                            "[Patrol] Cannot start — no valid path computed! "
                            "Add waypoints first."
                        )
                    else:
                        rospy.loginfo("=" * 60)
                        rospy.loginfo(
                            "[Patrol] ENTER pressed! Transitioning: RECORDING -> PATROLLING"
                        )
                        rospy.loginfo(
                            "[Patrol] Dispatching {} waypoints, {} path poses".format(
                                len(self.waypoints),
                                len(self.stitched_path.poses),
                            )
                        )
                        rospy.loginfo("=" * 60)

                        self.state = STATE_PATROLLING
                        self.arrive_count = 0
                        self.lap_count = 0
                        self._dispatch_path()

                elif self.state == STATE_PATROLLING:
                    rospy.loginfo(
                        "[Patrol] ENTER pressed during PATROLLING — "
                        "re-dispatching path (manual override)."
                    )
                    self._dispatch_path()

            # 持续发布预览路径和 Marker（保持 latch 刷新）
            if self.state == STATE_RECORDING and self.stitched_path is not None:
                # 降低发布频率，2秒刷新一次预览即可
                pass  # latch=True 已经保持了

            rate.sleep()


# ==================================================================
#                          主入口
# ==================================================================
if __name__ == "__main__":
    try:
        patrol = WaypointPatrol()
        patrol.run()
    except rospy.ROSInterruptException:
        pass