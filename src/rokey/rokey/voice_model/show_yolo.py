import os
import time
import sys
import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge



class YoloImageNode(Node):
    def __init__(self):
        super().__init__('yolo_image_node')

        # YOLO 모델 로드 (weights 파일 경로 지정)
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_text/resource/best_grand_final.pt')  # 원하는 가중치 파일로 변경

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        # 구독: 카메라 이미지
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 카메라 토픽 이름에 맞게 수정
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 발행: YOLO 결과 이미지
        self.publisher = self.create_publisher(
            Image,
            '/yolo/image',  # rqt_image_view로 볼 토픽
            10
        )

        self.angle_pub = self.create_publisher(
           Float32,
           '/bed_angle',
            10
       )


        self.get_logger().info('YOLO Image Node has started.')
    def mask_white_area_hsv(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower1 = np.array([90,   0,  80])
        upper1 = np.array([120, 90, 200])
        lower2 = np.array([149, 52, 65])
        upper2 = np.array([169,132,145])
        m1 = cv2.inRange(hsv, lower1, upper1)
        m2 = cv2.inRange(hsv, lower2, upper2)
        return cv2.bitwise_or(m1, m2)

    def get_blanket_angle_and_center(self, frame, x1, y1, x2, y2):
        margin = 0.15
        dx = int((x2-x1) * margin)
        dy = int((y2-y1) * margin)
        x1e = max(x1 - dx, 0)
        y1e = max(y1 - dy, 0)
        x2e = min(x2 + dx, frame.shape[1]-1)
        y2e = min(y2 + dy, frame.shape[0]-1)

        roi = frame[y1e:y2e, x1e:x2e]
        masked = self.mask_white_area_hsv(roi)
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours if cv2.contourArea(c) > 300]
        if not filtered:
            # 검출 실패 시 박스 중심
            return (x1+x2)//2, (y1+y2)//2, 0.0

        all_pts = np.vstack(filtered)
        (cx, cy), (w, h), angle = cv2.minAreaRect(all_pts)
        if w < h:
            angle += 90
        # ROI 기준 좌표를 원 영상 좌표로
        return cx + x1e, cy + y1e, angle
    
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV 이미지 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO 예측 수행
            results = self.model.predict(cv_image, imgsz=640, conf=0.5)

            bed_angle = None

            # 탐지 결과 이미지 그리기
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 바운딩 박스 정보
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())

                    class_name = self.model.names[cls]
                    label      = f'{class_name} {conf:.2f}'
                    # 박스 그리기
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    
                    if class_name == 'blanket':
                        _, _, angle = self.get_blanket_angle_and_center(cv_image, x1, y1, x2, y2)
                        bed_angle = float(angle)

            angle_msg = Float32(data=bed_angle if bed_angle is not None else 0.0)
            self.angle_pub.publish(angle_msg)

            # OpenCV 이미지 -> ROS Image 변환
            yolo_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # 퍼블리시
            self.publisher.publish(yolo_image_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
