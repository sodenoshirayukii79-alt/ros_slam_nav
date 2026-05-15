#!/usr/bin/env python3
# coding=utf-8
"""
color_detector.py  ——  感知层
基于 demo_cv_hsv.py，保留 trackbar / HSV 流程 / 形态学处理风格
修复：imshow 移至主线程；双重 for 循环改为 cv2.moments()
新增：发布 /ball_tracker/target (Point) 和 /ball_tracker/image (Image)
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

# ===== HSV 阈值初始值（与 demo_cv_hsv 保持一致）=====
hue_min  = 10
hue_max  = 40
satu_min = 90
satu_max = 255
val_min  = 1
val_max  = 255

bridge     = CvBridge()
pub_target = None
pub_image  = None

# ===== 全局图像缓冲：回调赋值，主线程 imshow =====
g_rgb    = None
g_hsv    = None
g_result = None


def Cam_RGB_Callback(msg):
    global hue_min, hue_max, satu_min, satu_max, val_min, val_max
    global pub_target, pub_image
    global g_rgb, g_hsv, g_result

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("格式转换错误: %s", e)
        return

    image_height, image_width = cv_image.shape[:2]

    # 将图片转换成 HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 在 HSV 空间做直方图均衡化
    h, s, v = cv2.split(hsv_image)
    v = cv2.equalizeHist(v)
    hsv_image = cv2.merge([h, s, v])

    # 使用 Hue / Saturation / Value 阈值范围对图像进行二值化
    th_image = cv2.inRange(hsv_image,
                           (hue_min, satu_min, val_min),
                           (hue_max, satu_max, val_max))

    # 开操作（去除噪点）
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    th_image = cv2.morphologyEx(th_image, cv2.MORPH_OPEN,  element)

    # 闭操作（连接连通域）
    th_image = cv2.morphologyEx(th_image, cv2.MORPH_CLOSE, element)

    # ===== 质心计算：cv2.moments() 替代双重 for 循环 =====
    target = Point()   # z == 0 表示未检测到目标

    contours, _ = cv2.findContours(th_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area > 500:
            M        = cv2.moments(largest)
            target_x = int(M["m10"] / M["m00"])
            target_y = int(M["m01"] / M["m00"])

            # 归一化坐标：图像中心为原点，范围 -1.0 ~ +1.0
            target.x = (target_x - image_width  / 2.0) / (image_width  / 2.0)
            target.y = (target_y - image_height / 2.0) / (image_height / 2.0)
            target.z = area / float(image_width * image_height)

            print(f"颜色质心坐标( {target_x} , {target_y} )  面积= {int(area)}")

            cv2.line(cv_image, (target_x - 10, target_y), (target_x + 10, target_y), (255, 0, 0), 2)
            cv2.line(cv_image, (target_x, target_y - 10), (target_x, target_y + 10), (255, 0, 0), 2)
        else:
            print("目标颜色消失...")
    else:
        print("目标颜色消失...")

    pub_target.publish(target)

    try:
        pub_image.publish(bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
    except CvBridgeError:
        pass

    # ===== 图像存入全局缓冲，主循环统一 imshow =====
    g_rgb    = cv_image
    g_hsv    = hsv_image
    g_result = th_image


def nothing(x):
    pass


if __name__ == "__main__":
    rospy.init_node("color_detector", anonymous=False)

    pub_target = rospy.Publisher("/ball_tracker/target", Point, queue_size=1)
    pub_image  = rospy.Publisher("/ball_tracker/image",  Image, queue_size=1)

    rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, Cam_RGB_Callback, queue_size=1)

    cv2.namedWindow("Threshold")
    cv2.createTrackbar("hue_min",  "Threshold", hue_min,  179, nothing)
    cv2.createTrackbar("hue_max",  "Threshold", hue_max,  179, nothing)
    cv2.createTrackbar("satu_min", "Threshold", satu_min, 255, nothing)
    cv2.createTrackbar("satu_max", "Threshold", satu_max, 255, nothing)
    cv2.createTrackbar("val_min",  "Threshold", val_min,  255, nothing)
    cv2.createTrackbar("val_max",  "Threshold", val_max,  255, nothing)
    cv2.namedWindow("RGB")
    cv2.namedWindow("HSV")
    cv2.namedWindow("Result")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # trackbar 读取（主线程）
        hue_min  = cv2.getTrackbarPos("hue_min",  "Threshold")
        hue_max  = cv2.getTrackbarPos("hue_max",  "Threshold")
        satu_min = cv2.getTrackbarPos("satu_min", "Threshold")
        satu_max = cv2.getTrackbarPos("satu_max", "Threshold")
        val_min  = cv2.getTrackbarPos("val_min",  "Threshold")
        val_max  = cv2.getTrackbarPos("val_max",  "Threshold")

        # ===== imshow 在主线程调用，GUI 事件正常处理 =====
        if g_rgb    is not None: cv2.imshow("RGB",    g_rgb)
        if g_hsv    is not None: cv2.imshow("HSV",    g_hsv)
        if g_result is not None: cv2.imshow("Result", g_result)
        cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()