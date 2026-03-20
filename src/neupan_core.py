#!/usr/bin/env python

"""
neupan_core.py — NeuPAN ROS 核心控制器 (Thread-Safe Refactored Version)

重构要点:
  1. 三通道线程安全缓冲区 (path / waypoints / goal) + threading.Lock
  2. 所有 planner 突变操作均在主线程 run() 中执行
  3. /neupan_arrived 持续状态广播
  4. 保留独立测试模式 (/neupan_goal) 的完整能力
"""

from neupan import neupan
import rospy
import threading
import numpy as np
from math import sin, cos, atan2

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from neupan.util import get_transform
import tf


class neupan_core:
    def __init__(self):

        rospy.init_node("neupan_node", anonymous=True)

        # ==================== ROS 参数 ====================
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_link")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_para, dtype=np.float32, sep=" "
        )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0, 5.0")
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")

        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param(
            "~include_initial_path_direction", False
        )

        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        # ==================== 初始化 NeuPAN Planner ====================
        pan = {"dune_checkpoint": self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file, pan=pan
        )

        # ==================== 运行时数据 ====================
        self.obstacle_points = None  # (2, n)
        self.robot_state = None      # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False

        # ==================== 线程安全缓冲区 (核心修复) ====================
        self._lock = threading.Lock()

        # 通道 1: 完整路径 (来自 /initial_path，巡航系统或 GlobalPlanner)
        self._path_buffer = None   # list of np.array (4,1)
        self._path_flag = False

        # 通道 2: 路点序列 (来自 /neupan_waypoints)
        self._wp_buffer = None     # list of np.array (4,1) 或 (3,1)
        self._wp_flag = False

        # 通道 3: 单目标点 (来自 /neupan_goal，独立测试模式)
        self._goal_buffer = None   # np.array (3,1)
        self._goal_flag = False

        # ==================== Publisher ====================
        self.vel_pub = rospy.Publisher("/neupan_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher("/neupan_ref_state", Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher("/neupan_initial_path", Path, queue_size=10)

        # [新增] 到达状态持续广播
        self.arrived_pub = rospy.Publisher("/neupan_arrived", Bool, queue_size=10)

        # RViz 可视化
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        # ==================== TF ====================
        self.listener = tf.TransformListener()

        # ==================== Subscriber ====================
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/initial_path", Path, self.path_callback)
        rospy.Subscriber("/neupan_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/neupan_goal", PoseStamped, self.goal_callback)

        rospy.loginfo("[neupan_core] Thread-safe refactored node initialized.")

    # ==================================================================
    #                          主控制循环
    # ==================================================================
    def run(self):

        r = rospy.Rate(50)

        # 频率统计
        last_freq_time = rospy.Time.now()
        loop_count = 0

        while not rospy.is_shutdown():

            # -------- 第 1 步: 主线程安全消费缓冲区 --------
            self._drain_buffers()

            # -------- 第 2 步: 获取机器人状态 (TF) --------
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, rospy.Time(0)
                )
                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf: {} -> {}".format(
                        self.base_frame, self.map_frame
                    ),
                )
                # 即使 TF 失败，仍然广播当前到达状态
                self.arrived_pub.publish(Bool(data=self.arrive))
                r.sleep()
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                self.arrived_pub.publish(Bool(data=self.arrive))
                r.sleep()
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            # -------- 第 3 步: 初始路径引导逻辑 --------
            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                self.neupan_planner.set_initial_path_from_state(self.robot_state)

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for neupan initial path")
                self.arrived_pub.publish(Bool(data=self.arrive))
                r.sleep()
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.neupan_planner.initial_path)
            )

            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1,
                    "No obstacle points, only path tracking task will be performed",
                )

            # -------- 第 4 步: NeuPAN MPC 规划 --------
            action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

            self.stop = info["stop"]
            self.arrive = info["arrive"]

            if info["arrive"]:
                rospy.loginfo_throttle(1.0, "arrive at the target")

            if info["stop"]:
                rospy.logwarn_throttle(
                    0.5,
                    "neupan stop triggered! Min dist: {:.2f}, Threshold: {:.2f}".format(
                        self.neupan_planner.min_distance.detach().item(),
                        self.neupan_planner.collision_threshold,
                    ),
                )

            # -------- 第 5 步: 发布控制量与可视化 --------
            self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
            self.vel_pub.publish(self.generate_twist_msg(action))

            dune_markers = self.generate_dune_points_markers_msg()
            if dune_markers is not None:
                self.point_markers_pub_dune.publish(dune_markers)

            nrmp_markers = self.generate_nrmp_points_markers_msg()
            if nrmp_markers is not None:
                self.point_markers_pub_nrmp.publish(nrmp_markers)

            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            # -------- 第 6 步: 持续广播到达状态 --------
            self.arrived_pub.publish(Bool(data=self.arrive))

            # -------- 第 7 步: 频率维持与统计 --------
            r.sleep()

            loop_count += 1
            now = rospy.Time.now()
            dt = (now - last_freq_time).to_sec()
            if dt >= 1.0:
                actual_freq = loop_count / dt
                rospy.loginfo(
                    "Neupan Control Loop Frequency: {:.2f} Hz".format(actual_freq)
                )
                if actual_freq < 40.0:
                    rospy.logwarn(
                        "Loop running SLOW! Target: 50Hz, Actual: {:.2f} Hz".format(
                            actual_freq
                        )
                    )
                last_freq_time = now
                loop_count = 0

    # ==================================================================
    #              缓冲区消费 (仅主线程调用，线程安全)
    # ==================================================================
    def _drain_buffers(self):
        """
        在主线程中安全地消费三个缓冲区。
        优先级: 完整路径 > 路点序列 > 单目标点
        一次循环最多处理一个通道，避免状态冲突。
        """
        with self._lock:
            # --- 优先级 1: 完整路径 (巡航系统 / GlobalPlanner) ---
            if self._path_flag:
                path_data = self._path_buffer
                self._path_buffer = None
                self._path_flag = False

                if path_data is not None and len(path_data) >= 2:
                    rospy.loginfo(
                        "[MAIN] Consuming path buffer ({} points)".format(
                            len(path_data)
                        )
                    )
                    if (
                        self.neupan_planner.initial_path is None
                        or self.refresh_initial_path
                    ):
                        self.neupan_planner.set_initial_path(path_data)
                        self.neupan_planner.reset()
                        self.arrive = False
                        self.stop = False
                        rospy.loginfo("[MAIN] Planner reset with new PATH.")
                return  # 一次只消费一个通道

            # --- 优先级 2: 路点序列 ---
            if self._wp_flag:
                wp_data = self._wp_buffer
                self._wp_buffer = None
                self._wp_flag = False

                if wp_data is not None and len(wp_data) >= 2:
                    rospy.loginfo(
                        "[MAIN] Consuming waypoints buffer ({} points)".format(
                            len(wp_data)
                        )
                    )
                    if (
                        self.neupan_planner.initial_path is None
                        or self.refresh_initial_path
                    ):
                        self.neupan_planner.update_initial_path_from_waypoints(wp_data)
                        self.neupan_planner.reset()
                        self.arrive = False
                        self.stop = False
                        rospy.loginfo("[MAIN] Planner reset with new WAYPOINTS.")
                return

            # --- 优先级 3: 单目标点 (独立测试模式) ---
            if self._goal_flag:
                goal_data = self._goal_buffer
                self._goal_buffer = None
                self._goal_flag = False

                if goal_data is not None and self.robot_state is not None:
                    rospy.loginfo(
                        "[MAIN] Consuming goal buffer: [{:.2f}, {:.2f}, {:.2f}]".format(
                            goal_data[0, 0], goal_data[1, 0], goal_data[2, 0]
                        )
                    )
                    self.neupan_planner.update_initial_path_from_goal(
                        self.robot_state, goal_data
                    )
                    self.neupan_planner.reset()
                    self.arrive = False
                    self.stop = False
                    rospy.loginfo("[MAIN] Planner reset with new GOAL.")
                return

    # ==================================================================
    #                     ROS 回调函数 (子线程)
    #          只做数据解析 + 存缓冲区 + 拉 Flag，绝不操作 planner
    # ==================================================================
    def scan_callback(self, scan_msg):
        """激光雷达回调 — 不涉及 planner，直接写 obstacle_points 是安全的"""
        if self.robot_state is None:
            return

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]
            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            return

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, rospy.Time(0)
            )
            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]
            trans_matrix, rot_matrix = get_transform(
                np.c_[x, y, yaw].reshape(3, 1)
            )
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return

    def path_callback(self, path_msg):
        """
        /initial_path 回调 (子线程)
        只做: 解析 Path -> list[np.array(4,1)] -> 存入缓冲区 -> 拉 Flag
        """
        if len(path_msg.poses) < 2:
            rospy.logwarn("[path_callback] Received path with < 2 poses, ignoring.")
            return

        initial_point_list = []
        for i in range(len(path_msg.poses)):
            p = path_msg.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y

            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                if i + 1 < len(path_msg.poses):
                    p2 = path_msg.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = (
                        initial_point_list[-1][2, 0] if initial_point_list else 0
                    )

            pts = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(pts)

        with self._lock:
            self._path_buffer = initial_point_list
            self._path_flag = True

        rospy.loginfo(
            "[path_callback] Buffered {} points -> waiting for main thread".format(
                len(initial_point_list)
            )
        )

    def waypoints_callback(self, path_msg):
        """
        /neupan_waypoints 回调 (子线程)
        只做: 解析 -> 存缓冲区 -> 拉 Flag
        """
        if self.robot_state is None:
            rospy.logwarn("[waypoints_callback] robot_state is None, ignoring.")
            return

        waypoints_list = [self.robot_state.copy()]
        for i in range(len(path_msg.poses)):
            p = path_msg.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                if i + 1 < len(path_msg.poses):
                    p2 = path_msg.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]
            pts = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(pts)

        with self._lock:
            self._wp_buffer = waypoints_list
            self._wp_flag = True

        rospy.loginfo(
            "[waypoints_callback] Buffered {} waypoints -> waiting for main thread".format(
                len(waypoints_list)
            )
        )

    def goal_callback(self, goal_msg):
        """
        /neupan_goal 回调 (子线程，独立测试模式)
        只做: 解析目标 -> 存缓冲区 -> 拉 Flag
        绝对不碰 planner!
        """
        x = goal_msg.pose.position.x
        y = goal_msg.pose.position.y
        theta = self.quat_to_yaw(goal_msg.pose.orientation)

        goal_array = np.array([[x], [y], [theta]])

        with self._lock:
            self._goal_buffer = goal_array
            self._goal_flag = True

        rospy.loginfo(
            "[goal_callback] Buffered goal [{:.2f}, {:.2f}, {:.2f}] -> waiting for main thread".format(
                x, y, theta
            )
        )

    # ==================================================================
    #                      消息生成工具函数
    # ==================================================================
    def generate_path_msg(self, path_list):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0
        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index
            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])
            path.poses.append(ps)
        return path

    def generate_twist_msg(self, vel):
        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        if self.stop or self.arrive:
            return Twist()
        else:
            action = Twist()
            action.linear.x = speed
            action.angular.z = steer
            return action

    def generate_dune_points_markers_msg(self):
        if self.neupan_planner.dune_points is None:
            return None
        marker_array = MarkerArray()
        points = self.neupan_planner.dune_points
        for index, point in enumerate(points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.seq = 0
            marker.header.stamp = rospy.get_rostime()
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0
            marker.color.r = 160.0 / 255.0
            marker.color.g = 32.0 / 255.0
            marker.color.b = 240.0 / 255.0
            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker_array.markers.append(marker)
        return marker_array

    def generate_nrmp_points_markers_msg(self):
        if self.neupan_planner.nrmp_points is None:
            return None
        marker_array = MarkerArray()
        points = self.neupan_planner.nrmp_points
        for index, point in enumerate(points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.seq = 0
            marker.header.stamp = rospy.get_rostime()
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 128.0 / 255.0
            marker.color.b = 0.0
            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.3
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker_array.markers.append(marker)
        return marker_array

    def generate_robot_marker_msg(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = 0
        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase
            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z
            marker.type = Marker.CUBE
            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]
            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y
            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(theta)
        return marker

    # ==================================================================
    #                      静态工具函数
    # ==================================================================
    def quat_to_yaw_list(self, quater):
        x, y, z, w = quater[0], quater[1], quater[2], quater[3]
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z ** 2 + y ** 2))
        return yaw

    @staticmethod
    def yaw_to_quat(yaw):
        quater = Quaternion()
        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)
        return quater

    @staticmethod
    def quat_to_yaw(quater):
        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w
        raw = atan2(2 * (w * z + x * y), 1 - 2 * (z ** 2 + y ** 2))
        return raw