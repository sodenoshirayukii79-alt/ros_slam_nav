#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
障碍物标记可视化节点
ROS1 MarkerArray 可视化节点

功能：
  - 使用线程锁保护共享激光缓存
  - 单次发布周期使用统一时间戳保证标记一致性
  - 根据前方障碍物动态调整危险扇区颜色
  - 避障箭头使用软映射计算角度
  - 激光点动态降采样（目标约 120 点，与雷达密度无关）
"""

import rospy
import math
import threading

from sensor_msgs.msg         import LaserScan
from geometry_msgs.msg       import Twist, Point, Quaternion
from std_msgs.msg            import ColorRGBA
from visualization_msgs.msg  import Marker, MarkerArray
from actionlib_msgs.msg      import GoalStatusArray


class ObstacleMarker:

    # ------------------------------------------------------------------ #
    # 命名空间常量
    # ------------------------------------------------------------------ #
    NS_LASER   = "laser_points"
    NS_ROBOT   = "robot_state"
    NS_NEAREST = "nearest_obstacle"
    NS_SAFETY  = "safety_zone"
    NS_ARROW   = "avoid_direction"
    NS_DANGER  = "danger_sector"

    # ------------------------------------------------------------------ #
    # 配置参数
    # ------------------------------------------------------------------ #
    SAFETY_DISTANCE     = 0.5
    DANGER_ANGLE_DEG    = 30.0
    MARKER_LIFETIME_SEC = 0.15   # 缩短生命周期减少视觉延迟
    FRONT_OBSTACLE_DIST = 1.0   # 只有小于此距离才算前方有效障碍物

    def __init__(self):
        rospy.init_node("obstacle_marker", anonymous=False)

        # 线程锁保护所有缓存的激光数据
        self._lock = threading.Lock()

        # 状态
        self.scan          = None
        self.cmd           = Twist()
        self.is_navigating = False

        # 缓存的扫描分析结果
        self.scan_points      = []
        self.nearest_point    = None
        self.nearest_distance = float('inf')
        self.front_obstacles  = []

        # 动态降采样步长（每次扫描重新计算）
        self._downsample_step = 3

        # 话题订阅
        rospy.Subscriber("/scan",             LaserScan,       self._scan_cb,   queue_size=1)
        rospy.Subscriber("/cmd_vel_raw",      Twist,           self._cmd_cb,    queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._status_cb, queue_size=1)

        # 话题发布
        self.pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=1)

        # 定时器驱动发布，20Hz 以获得更平滑的更新
        rospy.Timer(rospy.Duration(0.05), self._timer_cb)

    # ------------------------------------------------------------------ #
    # 回调函数
    # ------------------------------------------------------------------ #
    def _scan_cb(self, msg):
        # 根据实际点数重新计算降采样步长
        n = len(msg.ranges)
        step = max(1, n // 120)

        pts, nearest, nearest_d, front = self._compute_scan(msg)

        # 在锁保护下原子更新所有缓存字段
        with self._lock:
            self.scan             = msg
            self.scan_points      = pts
            self.nearest_point    = nearest
            self.nearest_distance = nearest_d
            self.front_obstacles  = front
            self._downsample_step = step

    def _cmd_cb(self, msg):
        self.cmd = msg

    def _status_cb(self, msg):
        self.is_navigating = any(s.status == 1 for s in msg.status_list)

    def _timer_cb(self, _):
        self.publish()

    # ------------------------------------------------------------------ #
    # 激光扫描分析（纯函数，无副作用，返回计算结果）
    # ------------------------------------------------------------------ #
    def _compute_scan(self, scan):
        """
        单次遍历激光扫描。返回：
          pts        — 所有有效返回点的 (x, y, r) 列表
          nearest    — 最近返回点的 (x, y, 0.0)，无则返回 None
          nearest_d  — 最近返回点的距离
          front      — 危险角度范围内的 (x, y, 0.0, r) 列表
        """
        pts       = []
        nearest   = None
        nearest_d = float('inf')
        front     = []

        danger_rad = math.radians(self.DANGER_ANGLE_DEG)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0 or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            pts.append((x, y, r))

            if r < nearest_d:
                nearest_d = r
                nearest   = (x, y, 0.0)

            if abs(angle) < danger_rad:
                front.append((x, y, 0.0, r))

        return pts, nearest, nearest_d, front

    # ------------------------------------------------------------------ #
    # 标记工厂
    # ------------------------------------------------------------------ #
    def _create_marker(self, marker_id, namespace, marker_type, stamp):
        # 由 publish() 传入的时间戳，所有标记保持一致
        m = Marker()
        m.header.frame_id    = "base_link"   
        m.header.stamp       = stamp
        m.ns                 = namespace
        m.id                 = marker_id
        m.type               = marker_type
        m.action             = Marker.ADD
        m.lifetime           = rospy.Duration(self.MARKER_LIFETIME_SEC)
        m.pose.orientation.w = 1.0
        m.points             = []
        m.colors             = []
        return m

    # ------------------------------------------------------------------ #
    # 标记构建器（均接收时间戳和缓存数据的本地副本）
    # ------------------------------------------------------------------ #
    def _build_laser_points(self, stamp, pts, step):
        m = self._create_marker(0, self.NS_LASER, Marker.POINTS, stamp)
        m.scale.x = 0.03
        m.scale.y = 0.03
        m.color.a = 1.0

        for x, y, r in pts[::step]:
            m.points.append(Point(x, y, 0.0))
            c = ColorRGBA(a=1.0)
            if r < 0.5:
                c.r, c.g, c.b = 1.0, 0.0, 0.0   # 红色：危险
            elif r < 1.0:
                c.r, c.g, c.b = 1.0, 1.0, 0.0   # 黄色：警告
            else:
                c.r, c.g, c.b = 0.0, 1.0, 0.0   # 绿色：安全
            m.colors.append(c)

        if not m.points:
            m.points.append(Point())
            m.colors.append(ColorRGBA(0, 0, 0, 0))

        return m

    def _build_robot_state_sphere(self, stamp, front_obstacles):
        m = self._create_marker(0, self.NS_ROBOT, Marker.SPHERE, stamp)
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.pose.position.z = 0.35

        # 只考虑距离小于阈值的有效障碍物
        close_obstacles = [obs for obs in front_obstacles if obs[3] < self.FRONT_OBSTACLE_DIST]

        if close_obstacles:
            # 红色：正在避障
            m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
        elif self.is_navigating:
            # 绿色：正常导航
            m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
        else:
            # 灰色：空闲
            m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5

        m.color.a = 0.95
        return m

    def _build_nearest_obstacle(self, stamp, nearest):
        m = self._create_marker(0, self.NS_NEAREST, Marker.SPHERE, stamp)
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.9

        if nearest is not None:
            m.pose.position.x = nearest[0]
            m.pose.position.y = nearest[1]
            m.pose.position.z = 0.05
        else:
            m.scale.x = m.scale.y = m.scale.z = 0.001

        return m

    def _build_safety_zone(self, stamp):
        m = self._create_marker(0, self.NS_SAFETY, Marker.CYLINDER, stamp)
        m.scale.x = m.scale.y = self.SAFETY_DISTANCE * 2
        m.scale.z = 0.02
        m.pose.position.z = 0.01
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.7, 1.0, 0.25
        return m

    def _build_avoidance_arrow(self, stamp):
        m = self._create_marker(0, self.NS_ARROW, Marker.ARROW, stamp)
        m.scale.x = 0.8
        m.scale.y = 0.08
        m.scale.z = 0.12
        m.pose.position.z = 0.5

        ang_z = self.cmd.angular.z

        if ang_z > 0.05:
            m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0  # 蓝色：左转
        elif ang_z < -0.05:
            m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # 红色：右转
        else:
            m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5  # 灰色：直行
        m.color.a = 0.8

        arrow_angle = math.copysign(math.pi / 2, ang_z) if abs(ang_z) > 0.05 else 0.0
        m.pose.orientation = self._yaw_to_quaternion(arrow_angle)
        return m

    def _build_danger_sector(self, stamp, scan, front_obstacles):
        m = self._create_marker(0, self.NS_DANGER, Marker.LINE_LIST, stamp)
        m.scale.x = 0.04

        # 前方有障碍物时扇区变红
        if front_obstacles:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.7

        if scan is None:
            return m

        danger_rad = math.radians(self.DANGER_ANGLE_DEG)
        R          = min(2.0, scan.range_max)
        origin     = Point(0.0, 0.0, 0.05)

        # Left boundary
        m.points += [origin,
                     Point(R * math.cos( danger_rad), R * math.sin( danger_rad), 0.05)]
        # Right boundary
        m.points += [origin,
                     Point(R * math.cos(-danger_rad), R * math.sin(-danger_rad), 0.05)]

        # Arc (20 segments)
        num_seg = 20
        for i in range(num_seg):
            a1 = -danger_rad + 2.0 * danger_rad * i       / num_seg
            a2 = -danger_rad + 2.0 * danger_rad * (i + 1) / num_seg
            m.points += [
                Point(R * math.cos(a1), R * math.sin(a1), 0.05),
                Point(R * math.cos(a2), R * math.sin(a2), 0.05),
            ]

        return m

    # ------------------------------------------------------------------ #
    # 工具函数
    # ------------------------------------------------------------------ #
    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ------------------------------------------------------------------ #
    # 发布
    # ------------------------------------------------------------------ #
    def publish(self):
        # 在锁保护下快照所有缓存数据，构建前释放锁
        with self._lock:
            if self.scan is None:
                return
            scan     = self.scan
            pts      = self.scan_points
            nearest  = self.nearest_point
            front    = self.front_obstacles
            step     = self._downsample_step

        # 整个帧使用单一时间戳
        now = rospy.Time.now()

        ma = MarkerArray()
        ma.markers.append(self._build_laser_points(now, pts, step))
        ma.markers.append(self._build_robot_state_sphere(now, front))
        ma.markers.append(self._build_nearest_obstacle(now, nearest))
        ma.markers.append(self._build_safety_zone(now))
        ma.markers.append(self._build_avoidance_arrow(now))
        ma.markers.append(self._build_danger_sector(now, scan, front))

        self.pub.publish(ma)


# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    ObstacleMarker()
    rospy.spin()

