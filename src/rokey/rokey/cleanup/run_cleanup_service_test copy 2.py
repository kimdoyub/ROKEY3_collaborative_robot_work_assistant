import rclpy
from rclpy.node import Node
from od_msg.srv import SrvDepthPosition
from std_msgs.msg import Bool
import os
import time
import sys
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation
from gtts import gTTS
import DR_init
from robot_control.onrobot import RG
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0
ANGLE_THRESHOLD = 45.0
BED_POSITION = [366.91, 88.86, 181.33, 41.85, -179.84, 41.94]
YOLO_MODEL_PATH = "/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_text/resource/best_grand_final.pt"

package_path = get_package_share_directory("pick_and_place_voice")

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, get_current_posj
from DR_common2 import posx, posj

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RunCleanupService(Node):
    def __init__(self):
        super().__init__('run_cleanup_service')
        self.srv = self.create_service(SrvDepthPosition, 'run_cleanup', self.handle_run_cleanup)
        self.llm_activation_pub = self.create_publisher(Bool, '/llm_activation', 10)
        self.get_logger().info('🧹 [run_cleanup] 서비스 대기 중...')

        self.init_robot()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None
        self.extraction_test = ["pillow", 'blanket', 'bed']

        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_rect_raw", self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.rgb_callback, 10)

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        self.get_position_request = SrvDepthPosition.Request()

    def init_robot(self):
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")

    def run_yolo_control(self):
        while rclpy.ok():
            if self.rgb_image is None:
                self.get_logger().warn("RGB image not received yet.")
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            frame = self.rgb_image.copy()
            results = self.model(frame)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy().astype(int)
            names = results.names

            bed_box = pillow_box = blanket_box = None
            angle = None

            for i, box in enumerate(boxes):
                label = names[classes[i]]
                x1, y1, x2, y2 = box.astype(int)
                if label == "bed":
                    bed_box = (x1, y1, x2, y2)
                elif label == "pillow":
                    pillow_box = (x1, y1, x2, y2)
                elif label == "blanket":
                    blanket_box = (x1, y1, x2, y2)
                    _, _, angle = self.get_blanket_angle_and_center(frame, x1, y1, x2, y2)

            if blanket_box and not bed_box and not pillow_box:
                self.handle_aligned_blanket_only()
            elif blanket_box and pillow_box and not bed_box:
                self.handle_blanket_and_pillow()
            elif blanket_box and pillow_box and bed_box:
                self.handle_messy_bed(pillow_box, bed_box, angle)
            break

    def handle_aligned_blanket_only(self):
        self.get_logger().info("정렬 완료 - 토닥이 동작")
        self.pat_motion(BED_POSITION)

    def handle_blanket_and_pillow(self):
        self.get_logger().info("이불 + 배개만 있음")
        self.pick_and_place("pillow")

    def handle_messy_bed(self, pillow_box, bed_box, angle):
        bed_center = (bed_box[0] + bed_box[2]) / 2
        pillow_center = (pillow_box[0] + pillow_box[2]) / 2

        if abs(bed_center - pillow_center) < 80:
            self.pick_and_place_pillow("pillow")
            if self.should_rotate(angle):
                self.rotate_gripper()
            self.pick_and_place("blanket")
            self.pick_and_place("pillow")
        else:
            if self.should_rotate(angle):
                self.rotate_gripper()
            self.pick_and_place("blanket")
            self.pat_motion(BED_POSITION)
            if abs(bed_center - pillow_center) < 80:
                self.rotate_gripper()
            self.pick_and_place("pillow")

    def pick_and_place(self, label):
        self.get_logger().info(f"픽 앤 플레이스 실행: {label}")
        self.pick_and_place_pillow(label) if label == "pillow" else None

    def pick_and_place_pillow(self, label):
        gripper.open_gripper()
        mwait()
        movel([400, 0, 200, 0, 180, 0], vel=VELOCITY, acc=ACC)  # 예시 위치
        gripper.close_gripper()
        mwait()
        movel(BED_POSITION, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pat_motion(self, pos):
        top = [pos[0], pos[1]-40, pos[2]-80, *pos[3:]]
        bottom = [pos[0], pos[1]-40, pos[2]-130, *pos[3:]]
        movel(top, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        for _ in range(3):
            movel(bottom, vel=VELOCITY, acc=ACC)
            movel(top, vel=VELOCITY, acc=ACC)
            top[1] += 40
            bottom[1] += 40

    def should_rotate(self, angle):
        return angle < 90 - ANGLE_THRESHOLD or angle > 90 + ANGLE_THRESHOLD

    def rotate_gripper(self):
        joints = get_current_posj()
        joints[5] += 90
        movej(joints, vel=50, acc=50)
        mwait()

    def get_blanket_angle_and_center(self, frame, x1, y1, x2, y2):
        margin = 0.15
        dx, dy = int((x2 - x1) * margin), int((y2 - y1) * margin)
        x1e, y1e = max(x1 - dx, 0), max(y1 - dy, 0)
        x2e, y2e = min(x2 + dx, frame.shape[1] - 1), min(y2 + dy, frame.shape[0] - 1)
        roi = frame[y1e:y2e, x1e:x2e]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([90, 0, 80]), np.array([120, 90, 200]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return (x1 + x2) // 2, (y1 + y2) // 2, 0
        all_pts = np.vstack(contours)
        rect = cv2.minAreaRect(all_pts)
        (cx, cy), (w, h), angle = rect
        if w < h:
            angle += 90
        return cx + x1e, cy + y1e, angle

    def handle_run_cleanup(self, request, response):
        self.get_logger().info(f"✅ [cleanup] 요청 수신: {request.result}")
        tts = gTTS(text=request.result, lang='ko')
        tts.save("goodbye.mp3")
        os.system("mpg123 goobbye.mp3")
        self.llm_activation_pub.publish(Bool(data=False))
        self.run_yolo_control()
        response.success = True
        response.feedback = "정리 작업 완료"
        return response

def main():
    node = RunCleanupService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
