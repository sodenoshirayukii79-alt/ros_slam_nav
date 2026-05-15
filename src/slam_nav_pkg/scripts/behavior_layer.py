#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from slam_nav_pkg.msg import ScanInfo
from std_msgs.msg import String


class BehaviorLayer:
    def __init__(self):
        rospy.init_node("behavior_layer", anonymous=False)

        self.cmd_vel_nav_topic      = rospy.get_param("~cmd_vel_nav_topic",      "/cmd_vel_nav")
        self.cmd_vel_avoid_topic    = rospy.get_param("~cmd_vel_avoid_topic",    "/cmd_vel_avoid")
        self.cmd_vel_recovery_topic = rospy.get_param("~cmd_vel_recovery_topic", "/cmd_vel_recovery")
        self.cmd_vel_teleop_topic   = rospy.get_param("~cmd_vel_teleop_topic",   "/cmd_vel_teleop")
        self.output_cmd_vel_topic   = rospy.get_param("~output_cmd_vel_topic",   "/cmd_vel")
        self.control_rate           = rospy.get_param("~control_rate",           10.0)

        self.inf_substitute         = rospy.get_param("~inf_substitute",         99.0)

        self.safe_distance_far      = rospy.get_param("~safe_distance_far",      0.5)
        self.safe_distance_warn     = rospy.get_param("~safe_distance_warn",     0.35)
        self.safe_distance_critical = rospy.get_param("~safe_distance_critical", 0.25)
        self.warn_scale             = rospy.get_param("~warn_scale",             0.8)
        self.critical_scale         = rospy.get_param("~critical_scale",         0.5)

        self.jerk_limit_linear  = rospy.get_param("~jerk_limit_linear",  0.5)
        self.jerk_limit_angular = rospy.get_param("~jerk_limit_angular", 1.0)
        self.ema_alpha_linear   = rospy.get_param("~ema_alpha_linear",   0.85)
        self.ema_alpha_angular  = rospy.get_param("~ema_alpha_angular",  0.85)
        self.deadband_linear    = rospy.get_param("~deadband_linear",    0.02)
        self.deadband_angular   = rospy.get_param("~deadband_angular",   0.03)

        self.teleop_timeout     = rospy.get_param("~teleop_timeout", 0.5)

        # ===== 状态 =====
        self.current_mode    = "NAV"
        self._prev_mode      = "NAV"
        self._prev_source    = "nav"
        self.last_teleop_time = rospy.Time(0)

        self.cmd_nav      = Twist()
        self.cmd_avoid    = Twist()
        self.cmd_recovery = Twist()
        self.cmd_teleop   = Twist()
        self.last_cmd     = Twist()
        self.filtered_cmd = Twist()

        # ===== 感知：直接用 scan_processor 的输出，不重复处理激光 =====
        self.scan_info = None
        rospy.Subscriber("/scan_info",                    ScanInfo, self.scan_info_callback,    queue_size=1)

        # ===== 速度指令订阅 =====
        rospy.Subscriber("/decision_mode",                String,   self.mode_callback,          queue_size=1)
        rospy.Subscriber(self.cmd_vel_nav_topic,          Twist,    self.cmd_nav_callback,       queue_size=1)
        rospy.Subscriber(self.cmd_vel_avoid_topic,        Twist,    self.cmd_avoid_callback,     queue_size=1)
        rospy.Subscriber(self.cmd_vel_recovery_topic,     Twist,    self.cmd_recovery_callback,  queue_size=1)
        rospy.Subscriber(self.cmd_vel_teleop_topic,       Twist,    self.cmd_teleop_callback,    queue_size=1)

        self.pub_cmd = rospy.Publisher(self.output_cmd_vel_topic, Twist, queue_size=1)

        rospy.loginfo("[BL] init | output=%s rate=%.0fHz", self.output_cmd_vel_topic, self.control_rate)

    # ===== 回调 =====
    def scan_info_callback(self, msg):
        self.scan_info = msg

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            rospy.logwarn("[BL] mode: %s → %s", self.current_mode, new_mode)
            self.current_mode = new_mode

    def cmd_nav_callback(self, msg):      self.cmd_nav = msg
    def cmd_avoid_callback(self, msg):    self.cmd_avoid = msg
    def cmd_recovery_callback(self, msg): self.cmd_recovery = msg

    def cmd_teleop_callback(self, msg):
        self.cmd_teleop = msg
        self.last_teleop_time = rospy.Time.now()

    # ===== 前方距离：直接从 ScanInfo 取，不重复计算 =====
    def _front_distance(self):
        if self.scan_info is None:
            return self.inf_substitute
        return self.scan_info.front_distance

    # ===== 安全减速 =====
    def _apply_safety(self, lin, ang, d):
        if d < self.safe_distance_critical:
            return 0.0, ang, 2
        if d < self.safe_distance_warn:
            return lin * self.critical_scale, ang, 1
        if d < self.safe_distance_far:
            return lin * self.warn_scale, ang, 1
        return lin, ang, 0

    # ===== 模式仲裁 =====
    def _select_intent(self):
        now = rospy.Time.now()
        if not self.last_teleop_time.is_zero() and \
                (now - self.last_teleop_time).to_sec() <= self.teleop_timeout:
            return self.cmd_teleop, "teleop"

        if self.current_mode == "RECOVERY":
            return self.cmd_recovery, "recovery"
        elif self.current_mode == "AVOID":
            return self.cmd_avoid, "avoid"
        elif self.current_mode == "NAV":
            return self.cmd_nav, "nav"
        return Twist(), "none"

    # ===== 加加速度限制 =====
    def _apply_jerk_limit(self, lin, ang):
        dlin = max(-self.jerk_limit_linear,  min(self.jerk_limit_linear,  lin - self.last_cmd.linear.x))
        dang = max(-self.jerk_limit_angular, min(self.jerk_limit_angular, ang - self.last_cmd.angular.z))
        return self.last_cmd.linear.x + dlin, self.last_cmd.angular.z + dang

    # ===== EMA 平滑 =====
    def _apply_ema(self, lin, ang):
        nav_lin = self.cmd_nav.linear.x if self.current_mode == "NAV" else 0.0
        stopping = abs(lin) < 0.01 and abs(nav_lin) < 0.01
        alpha_lin = 0.15 if stopping else self.ema_alpha_linear
        alpha_ang = 0.15 if stopping else self.ema_alpha_angular

        self.filtered_cmd.linear.x  = alpha_lin * lin + (1.0 - alpha_lin) * self.filtered_cmd.linear.x
        self.filtered_cmd.angular.z = alpha_ang * ang + (1.0 - alpha_ang) * self.filtered_cmd.angular.z

        if abs(self.filtered_cmd.linear.x) < 0.01:
            self.filtered_cmd.linear.x *= 0.5
        if abs(self.filtered_cmd.angular.z) < 0.01:
            self.filtered_cmd.angular.z *= 0.5

        return self.filtered_cmd.linear.x, self.filtered_cmd.angular.z

    # ===== 死区 =====
    def _deadband(self, lin, ang):
        if abs(lin) < self.deadband_linear:  lin = 0.0
        if abs(ang) < self.deadband_angular: ang = 0.0
        return lin, ang

    # ===== 主循环 =====
    def step(self):
        selected, source = self._select_intent()
        lin = selected.linear.x
        ang = selected.angular.z

        prev_nav = (self._prev_mode == "NAV")
        cur_nav  = (source == "nav")
        if (not prev_nav and cur_nav) or \
                (self._prev_source == "teleop" and source != "teleop"):
            self.filtered_cmd.linear.x  = 0.0
            self.filtered_cmd.angular.z = 0.0
            self.last_cmd.linear.x      = 0.0
            self.last_cmd.angular.z     = 0.0
            rospy.logwarn("[BL] reset EMA | %s → nav", self._prev_mode)

        self._prev_mode   = self.current_mode
        self._prev_source = source

        front_d = self._front_distance()
        lin, ang, safety_level = self._apply_safety(lin, ang, front_d)
        lin, ang = self._apply_jerk_limit(lin, ang)

        self.last_cmd.linear.x  = lin
        self.last_cmd.angular.z = ang

        lin, ang = self._apply_ema(lin, ang)
        lin, ang = self._deadband(lin, ang)

        out = Twist()
        out.linear.x  = lin
        out.angular.z = ang
        self.pub_cmd.publish(out)

        rospy.loginfo_throttle(0.2,
            "[BL] mode=%s src=%s lin=%.2f ang=%.2f front=%.2f safety=%d",
            self.current_mode, source, lin, ang, front_d, safety_level)

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    try:
        BehaviorLayer().run()
    except rospy.ROSInterruptException:
        pass