#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from slam_nav_pkg.msg import ScanInfo


class ScanProcessor:
    def __init__(self):
        rospy.init_node("scan_processor", anonymous=False)

        # ===== 参数加载 =====
        self.front_half_angle = rospy.get_param("~front_half_angle", 30.0)
        self.back_half_angle = rospy.get_param("~back_half_angle", 30.0)
        self.left_right_boundary = rospy.get_param("~left_right_boundary", 10.0)
        self.blocked_threshold = rospy.get_param("~blocked_threshold", 0.5)
        self.min_valid_range = rospy.get_param("~min_valid_range", 0.05)
        self.max_valid_range = rospy.get_param("~max_valid_range", 10.0)
        self.danger_max_dist = rospy.get_param("~danger_max_dist", 1.0)
        self.inf_substitute = rospy.get_param("~inf_substitute", 99.0)
        self.smoothing_window = max(1, int(rospy.get_param("~smoothing_window", 3)))
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self.scan_msg = None
        self.distance_history = {"front": [], "left": [], "right": [], "back": []}

        # ===== 话题订阅与发布 =====
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.pub_scan_info = rospy.Publisher("/scan_info", ScanInfo, queue_size=1)

    def scan_callback(self, msg):
        self.scan_msg = msg

    @staticmethod
    def _clip_index(i, n):
        return max(0, min(n - 1, i))

    def _angle_to_index(self, scan, angle_deg):
        angle_rad = math.radians(angle_deg)
        i = int((angle_rad - scan.angle_min) / scan.angle_increment)
        return self._clip_index(i, len(scan.ranges))

    def _is_valid(self, r):
        return math.isfinite(r) and self.min_valid_range <= r <= self.max_valid_range

    def _sector_min(self, scan, angle_min_deg, angle_max_deg):
        n = len(scan.ranges)
        if n == 0:
            return float("inf")

        i_min = self._angle_to_index(scan, angle_min_deg)
        i_max = self._angle_to_index(scan, angle_max_deg)
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        values = []
        for i in range(i_min, i_max + 1):
            r = scan.ranges[i]
            if self._is_valid(r):
                values.append(r)
        return min(values) if values else float("inf")

    def _smooth_mean(self, key, value):
        history = self.distance_history[key]
        history.append(value)
        if len(history) > self.smoothing_window:
            history.pop(0)

        finite_values = [v for v in history if math.isfinite(v)]
        if not finite_values:
            return float("inf")
        return sum(finite_values) / float(len(finite_values))

    def _calc_min_distance(self, scan):
        values = [r for r in scan.ranges if self._is_valid(r)]
        return min(values) if values else float("inf")

    def _danger_level(self, dist):
        if not math.isfinite(dist):
            return 0.0
        return max(0.0, min(1.0, 1.0 - dist / self.danger_max_dist))

    def _distance_or_substitute(self, dist):
        return dist if math.isfinite(dist) else self.inf_substitute

    @staticmethod
    def _obstacle_direction(front_blocked, left_blocked, right_blocked, back_blocked):
        if front_blocked:
            return "front"
        if left_blocked:
            return "left"
        if right_blocked:
            return "right"
        if back_blocked:
            return "back"
        return "clear"

    # ===== 单次处理 =====
    def process_once(self):
        if self.scan_msg is None:
            return

        scan = self.scan_msg
        ba = self.back_half_angle

        front = self._sector_min(scan, -self.front_half_angle, self.front_half_angle)
        left = self._sector_min(scan, self.left_right_boundary, 180.0 - self.left_right_boundary)
        right = self._sector_min(scan, -(180.0 - self.left_right_boundary), -self.left_right_boundary)

        back_l = self._sector_min(scan, 180.0 - ba, 180.0)
        back_r = self._sector_min(scan, -180.0, -(180.0 - ba))
        back = min(back_l, back_r)

        front = self._smooth_mean("front", front)
        left = self._smooth_mean("left", left)
        right = self._smooth_mean("right", right)
        back = self._smooth_mean("back", back)

        min_dist = self._calc_min_distance(scan)
        danger_level = self._danger_level(min_dist)

        front_blocked = front < self.blocked_threshold
        left_blocked = left < self.blocked_threshold
        right_blocked = right < self.blocked_threshold
        back_blocked = back < self.blocked_threshold

        msg = ScanInfo()
        msg.min_distance = self._distance_or_substitute(min_dist)
        msg.front_blocked = front_blocked
        msg.left_blocked = left_blocked
        msg.right_blocked = right_blocked
        msg.danger_level = danger_level
        msg.obstacle_direction = self._obstacle_direction(
            front_blocked, left_blocked, right_blocked, back_blocked
        )
        msg.front_distance = self._distance_or_substitute(front)
        msg.left_distance = self._distance_or_substitute(left)
        msg.right_distance = self._distance_or_substitute(right)
        msg.back_distance = self._distance_or_substitute(back)

        self.pub_scan_info.publish(msg)

    # ===== 主循环 =====
    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.process_once()
            rate.sleep()


if __name__ == "__main__":
    try:
        ScanProcessor().run()
    except rospy.ROSInterruptException:
        pass