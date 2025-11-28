#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from perception_yolo.msg import TrafficLightState
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2


class YoloDetectorNode:
    def __init__(self):
        rospy.init_node("yolo_detector", anonymous=True)
        self.bridge = CvBridge()

        # 加载模型
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5m", pretrained=True)
        self.model.conf = 0.25  # 降低置信度
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")

        # 图像订阅
        image_topic = rospy.get_param(
            "~image_topic",
            "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw",
        )
        self.sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )

        # 发布器
        self.light_pub = rospy.Publisher(
            "/perception/yolo/traffic_light", TrafficLightState, queue_size=10
        )
        self.debug_image_pub = rospy.Publisher("/yolo_debug_image", Image, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        h, w = cv_img.shape[:2]

        # ========= 图像预处理：裁剪 + 放大 =========
        top_crop = cv_img[0 : int(h * 0.6), int(w * 0.25) : int(w * 0.75)]  # 中上部
        scale = 2.0
        cropped_scaled = cv2.resize(top_crop, (0, 0), fx=scale, fy=scale)

        # YOLO 检测
        results = self.model(cropped_scaled)
        detections = results.xyxy[0].cpu().numpy()

        best_light_msg = None
        best_conf = 0.0

        for det in detections:
            xmin, ymin, xmax, ymax, conf, class_id = det[:6]
            class_name = self.model.names[int(class_id)]

            if class_name not in ["traffic light", "traffic_light"]:
                continue

            # 映射回原始图像坐标
            xmin = int(xmin / scale + w * 0.25)
            xmax = int(xmax / scale + w * 0.25)
            ymin = int(ymin / scale)
            ymax = int(ymax / scale)

            roi = cv_img[ymin:ymax, xmin:xmax]
            if roi.shape[0] < 5 or roi.shape[1] < 5:
                continue

            # ======= cares only about red lights =======
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # 红色mask（合并两段）
            mask_red1 = cv2.inRange(
                hsv, np.array([0, 70, 60]), np.array([10, 255, 255])
            )
            mask_red2 = cv2.inRange(
                hsv, np.array([170, 70, 60]), np.array([180, 255, 255])
            )
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            red_ratio = np.count_nonzero(mask_red) / (
                mask_red.shape[0] * mask_red.shape[1]
            )

            if red_ratio > 0.05:  # 阈值可以调优
                light_state = "red"
            else:
                light_state = "not_red"

            rospy.loginfo(
                f"YOLO trafficlight: red pixel ratio={red_ratio:.3f} -> {light_state}"
            )

            if conf > best_conf:
                best_conf = conf
                best_light_msg = TrafficLightState()
                best_light_msg.header = msg.header
                best_light_msg.state = light_state
                best_light_msg.confidence = float(conf)
                best_light_msg.center = Point(
                    x=(xmin + xmax) / 2, y=(ymin + ymax) / 2, z=0
                )

            # ====== 绘制调试框 ======
            cv2.rectangle(cv_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(
                cv_img,
                f"{light_state} {conf:.2f}",
                (xmin, ymin - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # 发布红绿灯状态
        if best_light_msg:
            self.light_pub.publish(best_light_msg)
            rospy.loginfo(f"🚦 红绿灯状态: {best_light_msg.state}")
        else:
            fallback = TrafficLightState()
            fallback.header = msg.header
            fallback.state = "unknown"
            fallback.confidence = 0.0
            fallback.center = Point()
            self.light_pub.publish(fallback)

        # 发布调试图像
        debug_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        debug_img.header = msg.header
        self.debug_image_pub.publish(debug_img)


if __name__ == "__main__":
    try:
        node = YoloDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
