#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
标定专用动态 TF 发布节点。

通过 dynamic_reconfigure (rqt_reconfigure 滑条) 实时调节
base_link -> livox_frame 的外参 (x y z roll pitch yaw),
并以 50Hz 持续广播 TF。RViz 中点云会随滑条实时移动。

调好后, 把日志里打印的 args 字符串直接抄进 scout_tf.launch 的
static_transform_publisher (顺序为 x y z yaw pitch roll parent child)。

欧拉角约定与 tf2 static_transform_publisher 一致:
quaternion_from_euler(roll, pitch, yaw) (axes='sxyz') == setRPY(roll,pitch,yaw)
"""
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from dynamic_reconfigure.server import Server
from neupan_ros.cfg import LidarCalibConfig


class LidarCalibTF(object):
    def __init__(self):
        rospy.init_node("lidar_calib_tf")

        self.parent_frame = rospy.get_param("~parent_frame", "base_link")
        self.child_frame = rospy.get_param("~child_frame", "livox_frame")

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.config = None

        # 启动 dynamic_reconfigure 服务 (rqt_reconfigure 里出现滑条)
        self.server = Server(LidarCalibConfig, self.reconfigure_cb)

        # 50Hz 持续广播 (即使没拖滑条也持续发, 保证 TF 时间戳新鲜)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_tf)

        rospy.loginfo("lidar_calib_tf 已启动: %s -> %s  (用 rqt_reconfigure 拖滑条调节)",
                      self.parent_frame, self.child_frame)

    def reconfigure_cb(self, config, level):
        self.config = config
        # 打印当前值, 并直接给出可抄进 scout_tf.launch 的 args 写法 (x y z yaw pitch roll)
        rospy.loginfo(
            "[标定值] x=%.5f y=%.5f z=%.5f roll=%.5f pitch=%.5f yaw=%.5f\n"
            "         scout_tf 写法 -> args=\"%.5f %.5f %.5f %.5f %.5f %.5f %s %s\"",
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw,
            config.x, config.y, config.z, config.yaw, config.pitch, config.roll,
            self.parent_frame, self.child_frame)
        return config

    def publish_tf(self, event):
        if self.config is None:
            return
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.config.x
        t.transform.translation.y = self.config.y
        t.transform.translation.z = self.config.z
        q = tft.quaternion_from_euler(self.config.roll, self.config.pitch, self.config.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(t)


if __name__ == "__main__":
    try:
        LidarCalibTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
