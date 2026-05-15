#!/usr/bin/env python3
# coding=utf-8
"""
color_follower.py  ——  控制层
基于 demo_cv_follow.py 的速度计算公式
订阅 color_detector 发布的归一化质心，输出速度到 /cmd_vel_follow
不订阅相机，不做图像处理，职责单一
"""

import rospy
from geometry_msgs.msg import Point, Twist

# kinect2 qhd 分辨率，用于将归一化坐标还原为像素偏移
# 与 demo_cv_follow 的速度系数保持一致
IMAGE_WIDTH  = 960
IMAGE_HEIGHT = 540

vel_cmd = Twist()
vel_pub = None


def Target_Callback(msg):
    """
    订阅 /ball_tracker/target (Point)
      msg.x: 归一化水平偏移 -1.0(左) ~ +1.0(右)
      msg.y: 归一化垂直偏移 -1.0(上) ~ +1.0(下)
      msg.z: 面积比，0.0 表示未检测到目标
    速度计算还原为像素坐标后套用 demo_cv_follow 原始公式
    """
    global vel_cmd, vel_pub

    if msg.z == 0.0:
        print("目标颜色消失...")
        vel_cmd.linear.x  = 0
        vel_cmd.angular.z = 0
    else:
        # 归一化坐标 → 像素坐标（与 demo_cv_follow 公式对齐）
        target_x = msg.x * (IMAGE_WIDTH  / 2.0) + IMAGE_WIDTH  / 2.0
        target_y = msg.y * (IMAGE_HEIGHT / 2.0) + IMAGE_HEIGHT / 2.0

        vel_forward       = (IMAGE_HEIGHT / 2 - target_y) * 0.001
        vel_turn          = (IMAGE_WIDTH  / 2 - target_x) * 0.0005
        vel_cmd.linear.x  = vel_forward
        vel_cmd.angular.z = vel_turn

        print(f"颜色质心坐标( {int(target_x)} , {int(target_y)} )")
        print(f"机器人运动速度( linear.x= {vel_cmd.linear.x:.3f} , angular.z= {vel_cmd.angular.z:.3f} )")

    vel_pub.publish(vel_cmd)


if __name__ == "__main__":
    rospy.init_node("color_follower", anonymous=False)

    # 输出到 /cmd_vel_follow，由 launch 里的 relay 转发到 /cmd_vel
    vel_pub = rospy.Publisher("/cmd_vel_follow", Twist, queue_size=1)

    rospy.Subscriber("/ball_tracker/target", Point, Target_Callback, queue_size=1)

    rospy.spin()