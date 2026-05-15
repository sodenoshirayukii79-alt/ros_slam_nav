#!/usr/bin/env python3
# coding=utf-8
"""
face_detector.py  ——  感知层
基于 demo_cv_face_detect.py，保留 Haar Cascade 检测流程
修复：imshow 移至主线程（避免黑屏）
新增：发布 /face_tracker/target (Point) 和 /face_tracker/image (Image)
      取面积最大人脸（最近），其余人脸灰色框标注
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

bridge       = CvBridge()
pub_target   = None
pub_image    = None
face_cascade = None

# ===== 全局图像缓冲：回调赋值，主线程 imshow =====
g_face_img = None


def Cam_RGB_Callback(msg):
    global pub_target, pub_image, face_cascade
    global g_face_img

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("格式转换错误: %s", e)
        return

    image_height, image_width = cv_image.shape[:2]

    # 转换为灰度图
    gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # 直方图均衡化（提升暗环境检测率，与 demo_cv_hsv 风格一致）
    gray_img = cv2.equalizeHist(gray_img)

    # 人脸检测
    faces = face_cascade.detectMultiScale(gray_img, 1.3, 5)

    target = Point()   # z == 0.0 表示未检测到人脸

    if len(faces) > 0:
        # 取面积最大的人脸（距离最近）
        x, y, w, h = max(faces, key=lambda r: r[2] * r[3])

        target_x = x + w // 2
        target_y = y + h // 2
        target.x = (target_x - image_width  / 2.0) / (image_width  / 2.0)
        target.y = (target_y - image_height / 2.0) / (image_height / 2.0)
        target.z = float(w * h) / float(image_width * image_height)

        print(f"人脸质心坐标( {target_x} , {target_y} )  共检测到 {len(faces)} 张")

        # 最大人脸红色框，其余灰色框（与 demo_cv_face_detect 绘制风格一致）
        for (ox, oy, ow, oh) in faces:
            color = (0, 0, 255) if (ox == x and oy == y) else (180, 180, 180)
            cv2.rectangle(cv_image, (ox, oy), (ox + ow, oy + oh), color, 3)

        cv2.line(cv_image, (target_x - 10, target_y), (target_x + 10, target_y), (255, 0, 0), 2)
        cv2.line(cv_image, (target_x, target_y - 10), (target_x, target_y + 10), (255, 0, 0), 2)
    else:
        print("未检测到人脸...")

    pub_target.publish(target)

    try:
        pub_image.publish(bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
    except CvBridgeError:
        pass

    # ===== 图像存入全局缓冲，主循环统一 imshow =====
    g_face_img = cv_image


if __name__ == "__main__":
    rospy.init_node("face_detector", anonymous=False)

    # 级联分类器路径（ROS param 可覆盖，与 demo_cv_face_detect 路径参数化方式一致）
    cascade_path = rospy.get_param(
        "~cascade_path",
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
    )
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        rospy.logerr("无法加载级联分类器: %s", cascade_path)
        exit(1)

    pub_target = rospy.Publisher("/face_tracker/target", Point, queue_size=1)
    pub_image  = rospy.Publisher("/face_tracker/image",  Image, queue_size=1)

    rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, Cam_RGB_Callback, queue_size=1)

    cv2.namedWindow("face window")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # ===== imshow 在主线程调用 =====
        if g_face_img is not None:
            cv2.imshow("face window", g_face_img)
        cv2.waitKey(1)
        rate.sleep()

    cv2.destroyAllWindows()