#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
决策层 - 模式控制器
==================
输入：/scan_info（激光感知）+ /face_tracker/target（视觉感知）
输出：NAV / AVOID / RECOVERY 三状态

人脸感知辅助：
  face_detector 检测到人脸（z > 0）+ 激光判定为 NAV
  → 提前切入 AVOID，比纯激光更早减速
  → 人脸消失后正常切回 NAV（带防抖）
"""

import rospy
import dynamic_reconfigure.client
import std_srvs.srv
from geometry_msgs.msg import Twist, Point
from slam_nav_pkg.msg import ScanInfo
from std_msgs.msg import String


class DecisionNode:

    MODE_NAV      = "NAV"
    MODE_AVOID    = "AVOID"
    MODE_RECOVERY = "RECOVERY"

    def __init__(self):
        rospy.init_node('decision_node', anonymous=False)

        # ===== 激光感知参数 =====
        self.threshold_avoid        = rospy.get_param('~threshold_avoid',        0.5)
        self.threshold_recovery     = rospy.get_param('~threshold_recovery',     0.3)
        self.debounce_time          = rospy.get_param('~debounce_time',          0.5)
        self.recovery_min_duration  = rospy.get_param('~recovery_min_duration',  3.0)
        self.reconfigure_throttle   = rospy.get_param('~reconfigure_throttle',   1.0)
        self.cmd_vel_nav_topic      = rospy.get_param('~cmd_vel_nav_topic',      '/cmd_vel_nav')
        self.cmd_vel_avoid_topic    = rospy.get_param('~cmd_vel_avoid_topic',    '/cmd_vel_avoid')
        self.cmd_vel_recovery_topic = rospy.get_param('~cmd_vel_recovery_topic', '/cmd_vel_recovery')

        # ===== 人脸感知参数 =====
        self.face_timeout = rospy.get_param('~face_timeout', 2.0)

        # ===== TEB 参数 =====
        self.teb_params = {
            self.MODE_NAV: {
                'max_vel_x':         rospy.get_param('~nav_max_vel_x',           0.5),
                'min_obstacle_dist': rospy.get_param('~nav_min_obstacle_dist',   0.4),
                'weight_obstacle':   rospy.get_param('~nav_weight_obstacle',     50.0),
            },
            self.MODE_AVOID: {
                'max_vel_x':         rospy.get_param('~avoid_max_vel_x',         0.2),
                'min_obstacle_dist': rospy.get_param('~avoid_min_obstacle_dist', 0.18),
                'weight_obstacle':   rospy.get_param('~avoid_weight_obstacle',   80.0),
            },
            self.MODE_RECOVERY: {
                'max_vel_x':         rospy.get_param('~recovery_max_vel_x',      0.05),
                'weight_obstacle':   rospy.get_param('~recovery_weight_obstacle', 100.0),
            }
        }
        self.teb_namespace = rospy.get_param('~teb_namespace', '/move_base/TebLocalPlannerROS')

        # ===== 状态变量 =====
        self.current_mode          = self.MODE_NAV
        self.latest_scan_info      = None
        self.latest_nav_cmd        = Twist()
        self.pending_mode          = None
        self.mode_start_time       = rospy.Time.now()
        self.recovery_enter_time   = rospy.Time(0)
        self.last_reconfigure_time = rospy.Time(0)
        self.reconfigure_pending   = False
        self.pending_params        = None
        self.mode_switch_count     = 0
        self.last_log_time         = rospy.Time(0)

        # ===== 人脸状态 =====
        self.face_detected  = False
        self.face_last_time = rospy.Time(0)

        # ===== 订阅 =====
        rospy.Subscriber('/scan_info',           ScanInfo, self.scan_info_callback, queue_size=1)
        rospy.Subscriber(self.cmd_vel_nav_topic, Twist,    self.cmd_vel_nav_callback, queue_size=1)
        # 视觉感知：face_detector 30Hz 发布，z>0 有人脸，z==0 无人脸
        rospy.Subscriber('/face_tracker/target', Point,    self.face_cb, queue_size=1)

        # ===== 发布 =====
        self.pub_mode             = rospy.Publisher('/decision_mode',            String, queue_size=1)
        self.pub_cmd_vel_avoid    = rospy.Publisher(self.cmd_vel_avoid_topic,    Twist,  queue_size=1)
        self.pub_cmd_vel_recovery = rospy.Publisher(self.cmd_vel_recovery_topic, Twist,  queue_size=1)

        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)

        self.teb_client = None
        self._init_teb_client()

        rospy.loginfo("[DN] 初始化 | avoid<%.2fm | recovery<%.2fm | face_timeout=%.1fs",
                      self.threshold_avoid, self.threshold_recovery, self.face_timeout)

    # ===== TEB =====
    def _init_teb_client(self):
        try:
            self.teb_client = dynamic_reconfigure.client.Client(
                self.teb_namespace, timeout=5.0, config_callback=lambda c: None)
            rospy.loginfo("[DN] TEB 已连接")
        except Exception as e:
            rospy.logwarn("[DN] TEB 连接失败: %s", str(e))
            self.teb_client = None

    # ===== 人脸回调 =====
    def face_cb(self, msg):
        if msg.z > 0.0:
            if not self.face_detected:
                rospy.logwarn("[DN] 检测到人脸，提前触发 AVOID")
            self.face_detected  = True
            self.face_last_time = rospy.Time.now()
        else:
            self.face_detected = False

    # ===== 距离 → 模式 =====
    def _get_mode_from_distance(self, dist):
        if dist < self.threshold_recovery:
            return self.MODE_RECOVERY
        if dist < self.threshold_avoid:
            return self.MODE_AVOID
        return self.MODE_NAV

    def cmd_vel_nav_callback(self, msg):
        self.latest_nav_cmd = msg

    # ===== 激光 + 人脸双输入决策 =====
    def scan_info_callback(self, msg):
        self.latest_scan_info = msg
        front_ok = (not msg.front_blocked) and (msg.front_distance >= self.threshold_avoid)

        if front_ok:
            new_mode = self.MODE_NAV
        else:
            effective = min(msg.front_distance, msg.min_distance)
            new_mode  = self._get_mode_from_distance(effective)

        # ===== 人脸辅助：激光判 NAV 但视觉检测到人 → 提前切 AVOID =====
        if self.face_detected and new_mode == self.MODE_NAV:
            new_mode = self.MODE_AVOID
            rospy.loginfo_throttle(2.0, "[DN] 人脸感知上调 NAV → AVOID")

        self._try_switch(new_mode, msg)

    # ===== 防抖 =====
    def _try_switch(self, new_mode, scan_info):
        if new_mode == self.current_mode:
            self.pending_mode = None
            return

        if (self.current_mode == self.MODE_RECOVERY
                and new_mode != self.MODE_RECOVERY
                and not self.recovery_enter_time.is_zero()):
            if (rospy.Time.now() - self.recovery_enter_time).to_sec() < self.recovery_min_duration:
                return

        if new_mode != self.pending_mode:
            self.pending_mode    = new_mode
            self.mode_start_time = rospy.Time.now()
            rospy.loginfo("[DN] 待切换: %s → %s (防抖 %.1fs)",
                          self.current_mode, new_mode, self.debounce_time)
            return

        if (rospy.Time.now() - self.mode_start_time).to_sec() >= self.debounce_time:
            self._switch_mode(new_mode, scan_info)

    # ===== 执行切换 =====
    def _switch_mode(self, new_mode, scan_info):
        old_mode = self.current_mode
        self.current_mode      = new_mode
        self.pending_mode      = None
        self.mode_switch_count += 1

        if new_mode == self.MODE_RECOVERY:
            self.recovery_enter_time = rospy.Time.now()

        rospy.logwarn("[DN] 切换 #%d: %s → %s | front=%.2f min=%.2f face=%s",
                      self.mode_switch_count, old_mode, new_mode,
                      scan_info.front_distance, scan_info.min_distance,
                      "是" if self.face_detected else "否")

        if old_mode == self.MODE_RECOVERY and new_mode == self.MODE_NAV:
            try:
                self.clear_costmaps_srv()
                rospy.loginfo("[DN] RECOVERY→NAV: 代价地图已清除")
            except rospy.ServiceException as e:
                rospy.logwarn("[DN] clear_costmaps 失败: %s", str(e))

        self._apply_teb_params(new_mode)
        self.pub_mode.publish(String(data=new_mode))

    # ===== TEB 参数应用 =====
    def _apply_teb_params(self, mode):
        if self.teb_client is None:
            self._init_teb_client()
            if self.teb_client is None:
                rospy.logerr("[DN] TEB 不可用")
                return
        params  = self.teb_params[mode]
        elapsed = (rospy.Time.now() - self.last_reconfigure_time).to_sec()
        if elapsed < self.reconfigure_throttle:
            self.reconfigure_pending = True
            self.pending_params      = params
            return
        self._do_reconfigure(params)

    def _do_reconfigure(self, params):
        try:
            min_obs = params.get('min_obstacle_dist', None)
            rospy.loginfo("[DN] TEB: vx=%.2f obs=%s w=%.0f",
                          params['max_vel_x'],
                          "keep" if min_obs is None else "%.2f" % min_obs,
                          params['weight_obstacle'])
            self.teb_client.update_configuration(params)
            self.last_reconfigure_time = rospy.Time.now()
            self.reconfigure_pending   = False
            self.pending_params        = None
        except Exception as e:
            rospy.logerr("[DN] TEB 重配置失败: %s", str(e))
            self.reconfigure_pending = True
            self.pending_params      = params

    def _check_pending_reconfigure(self):
        if self.reconfigure_pending and self.pending_params is not None:
            if (rospy.Time.now() - self.last_reconfigure_time).to_sec() >= self.reconfigure_throttle:
                self._do_reconfigure(self.pending_params)

    # ===== face_detector 崩溃超时保护 =====
    def _check_face_timeout(self):
        if (self.face_detected
                and not self.face_last_time.is_zero()
                and (rospy.Time.now() - self.face_last_time).to_sec() > self.face_timeout):
            rospy.logwarn("[DN] /face_tracker/target 超时 %.1fs，重置人脸状态", self.face_timeout)
            self.face_detected = False

    def _log_status(self):
        now = rospy.Time.now()
        if (now - self.last_log_time).to_sec() >= 5.0:
            self.last_log_time = now
            rospy.loginfo("[DN] 模式=%s 切换=%d 人脸=%s pending=%s",
                          self.current_mode, self.mode_switch_count,
                          "是" if self.face_detected else "否",
                          str(self.pending_mode))

    # ===== 速度意图 =====
    def _publish_velocity_intents(self):
        avoid_cmd    = Twist()
        recovery_cmd = Twist()

        if self.current_mode == self.MODE_AVOID and self.latest_scan_info is not None:
            avoid_cmd.linear.x = min(self.latest_nav_cmd.linear.x, 0.10)
            diff = self.latest_scan_info.left_distance - self.latest_scan_info.right_distance
            avoid_cmd.angular.z = max(-0.5, min(0.5, diff * 0.8))

        if self.current_mode == self.MODE_RECOVERY:
            if self.latest_scan_info is not None:
                back_d = getattr(self.latest_scan_info, 'back_distance', float('inf'))
                recovery_cmd.linear.x  = -0.10 if back_d > 0.4 else 0.0
                l = self.latest_scan_info.left_distance
                r = self.latest_scan_info.right_distance
                recovery_cmd.angular.z = 0.8 if l > r else -0.8
            else:
                recovery_cmd.linear.x  = -0.10
                recovery_cmd.angular.z = 0.8

        self.pub_cmd_vel_avoid.publish(avoid_cmd)
        self.pub_cmd_vel_recovery.publish(recovery_cmd)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._check_pending_reconfigure()
            self._check_face_timeout()
            self._publish_velocity_intents()
            self._log_status()
            rate.sleep()


if __name__ == '__main__':
    try:
        DecisionNode().run()
    except rospy.ROSInterruptException:
        pass