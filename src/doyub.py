import os
import time
import sys
import cv2
import numpy as np
from ultralytics import YOLO
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from robot_control.onrobot import RG
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ================= 기본 설정 =================
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

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, movec, move_periodic,get_current_posj
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
# ================= 로봇 제어 클래스 =================
class RobotController(Node):

    def __init__(self):
        super().__init__("yolo_robot_controller")
        self.init_robot()
        self.model = YOLO(YOLO_MODEL_PATH)

        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_sub = self.create_subscription(
            Image,
            "/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")

        
        self.get_position_request = SrvDepthPosition.Request()
            
        self.rgb_image = None
        

        self.rgb_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",  
            self.rgb_callback,
            10
        )
        self.extraction_test = ["pillow", 'blanket', 'bed']

        
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

    def get_depth_from_sensor(self, x, y):
        if self.depth_image is None:
            self.get_logger().warn("Depth image not received yet.")
            return 0.0

        h, w = self.depth_image.shape
        x = int(np.clip(x, 0, w - 1))
        y = int(np.clip(y, 0, h - 1))

        depth = self.depth_image[y, x]
        if np.isnan(depth) or depth == 0:
            self.get_logger().warn(f"Invalid depth at ({x}, {y})")
            return 0.0

        return float(depth)

    def convert_pixel_to_3d(self, x, y, depth, fx=600, fy=600, cx=320, cy=240):
        X = (x - cx) * depth / fx
        Y = (y - cy) * depth / fy
        Z = depth
        return [X, Y, Z]

    def mask_white_area_hsv(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([90, 0, 80])
        upper_white = np.array([120, 90, 200])
        lower2 = np.array([149, 52, 65])
        upper2 = np.array([169, 132, 145])
        mask1 = cv2.inRange(hsv, lower_white, upper_white)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return cv2.bitwise_or(mask1, mask2)


    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T
    
    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def pick_and_place(self, target_name):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        
        if target_name:
            self.target_pos = self.get_target_pos(target_name)

        approach_pos = self.target_pos.copy()
        approach_pos[2] += 120  # z축 위로 접근

        movel(approach_pos, vel=VELOCITY, acc=ACC)
        movel(self.target_pos, vel=VELOCITY, acc=ACC)

        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        self.target_pos[2]+=120
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        movel(BED_POSITION, vel=VELOCITY, acc=ACC)
        # movel(posx(515.02, -92.82, 286.32, -124.65, 150.51, -94.51), vel=VELOCITY, acc=ACC)        
        let = [BED_POSITION[0], BED_POSITION[1], BED_POSITION[2]-120, *BED_POSITION[3:]]           
        movel(let, vel=20, acc=ACC)
        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def pick_and_place_pillow(self, target_name):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

        if target_name:
            self.target_pos = self.get_target_pos(target_name)

        approach_pos = self.target_pos.copy()
        approach_pos[2] += 120  # z축 위로 접근

        movel(approach_pos, vel=VELOCITY, acc=ACC)
        movel(self.target_pos, vel=VELOCITY, acc=ACC)

        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        self.target_pos[2]+=120
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        temp_leave = BED_POSITION.copy()
        temp_leave[1] += 100
        movel(temp_leave, vel=VELOCITY, acc=ACC)
        # movel(posx(515.02, -92.82, 286.32, -124.65, 150.51, -94.51), vel=VELOCITY, acc=ACC)        
        let = [temp_leave[0], temp_leave[1], temp_leave[2]-120, *temp_leave[3:]]           
        movel(let, vel=20, acc=ACC)
        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def init_robot(self):
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()



    def pat_motion(self, pos):
        ##########################토닥이기 위 아래 좌표 잡기##################
        top = [pos[0], pos[1]-40, pos[2]-80, *pos[3:]]
        bottom = [pos[0], pos[1]-40, pos[2] -130, *pos[3:]]
        movel(top, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()
        mwait()
        for _ in range(3):
            movel(bottom, vel=VELOCITY, acc=ACC)
            mwait()
            movel(top, vel=VELOCITY, acc=ACC)
            mwait()
            top[1] += 40
            bottom[1] += 40

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET  # z값 보정
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # 최소 깊이 보장

            target_pos = list(td_coord[:3]) + robot_posx[3:]  # 위치 + 자세
        return target_pos

    def run_yolo_control(self):
        while rclpy.ok():
            if self.rgb_image is None:
                self.get_logger().warn("RGB image not received yet.")
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            frame = self.rgb_image.copy()  # 최신 프레임 복사
            results = self.model(frame)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy().astype(int)
            names = results.names

            bed_box = pillow_box = blanket_box = None
            center = angle = None

            for i, box in enumerate(boxes):
                label = names[classes[i]]
                x1, y1, x2, y2 = box.astype(int)
                if label == "bed":
                    bed_box = (x1, y1, x2, y2)
                elif label == "pillow":
                    pillow_box = (x1, y1, x2, y2)
                elif label == "blanket":
                    blanket_box = (x1, y1, x2, y2)
                    cx, cy, angle = self.get_blanket_angle_and_center(frame, x1, y1, x2, y2)

            if blanket_box and not bed_box and not pillow_box:
                print("정렬완료")
                self.pat_motion(BED_POSITION)
                break
            

            elif blanket_box and pillow_box and not bed_box:
                print("이불만 침대 위에")

                bed_center_x = (blanket_box[0] + blanket_box[2])/2
                pillow_center_x = (pillow_box[0]+pillow_box[2])/2

                if abs(bed_center_x - pillow_center_x) < 80:
                    current_joints = get_current_posj()  # [j1, j2, j3, j4, j5, j6]
                    # 마지막 조인트(j6)만 변경
                    target_joints = current_joints.copy()
                    target_joints[5] += 45  # 예: 30도 회전 (상황에 따라 -도 가능)
                    target_joints[5] += 45 

                self.pick_and_place(self.extraction_test[0])

            elif blanket_box and pillow_box and bed_box:
                print("다 어질러짐")
                px = (pillow_box[0] + pillow_box[2]) // 2
                py = (pillow_box[1] + pillow_box[3]) // 2
                bx1, by1, bx2, by2 = bed_box

                if bx1 <= px <= bx2 and by1 <= py <= by2:
                    print("배개는 침대 위에")
                    self.pick_and_place_pillow(self.extraction_test[0])
                    self.pick_and_place(self.extraction_test[1])

                    print("이불 ")
                    if angle < 90 - ANGLE_THRESHOLD or angle > 90 + ANGLE_THRESHOLD:
                        #############그리퍼 방향 돌리기#############
                        # blanket_pos[5] += 90
                        current_joints = get_current_posj()  # [j1, j2, j3, j4, j5, j6]

                        # 마지막 조인트(j6)만 변경
                        target_joints = current_joints.copy()
                        target_joints[5] += 45  # 예: 30도 회전 (상황에 따라 -도 가능)
                        target_joints[5] += 45 

                        # 이동 명령
                        movej(target_joints, vel=50, acc=50)
                        mwait()
                        print("돌려")
                    print("집기")
                    self.pick_and_place(self.extraction_test[1])
                    mwait()
                    movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
                    mwait()

                    print("배개 집기")
                    self.pick_and_place(self.extraction_test[1])
                    movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
                    mwait()
                    break

                else:
                    print("배개도 침대 아래")
                    
                    print("이불 ")
                    if angle < 90 - ANGLE_THRESHOLD or angle > 90 + ANGLE_THRESHOLD:
                        #############그리퍼 방향 돌리기#############
                        # blanket_pos[5] += 90
                        current_joints = get_current_posj()  # [j1, j2, j3, j4, j5, j6]

                        # 마지막 조인트(j6)만 변경
                        target_joints = current_joints.copy()
                        target_joints[5] += 45  # 예: 30도 회전 (상황에 따라 -도 가능)
                        target_joints[5] += 45 

                        # 이동 명령
                        movej(target_joints, vel=50, acc=50)
                        mwait()

                        print("돌려")

                    print("집기")
                    self.pick_and_place(self.extraction_test[1])
                    self.pat_motion(BED_POSITION)
                    movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
                    mwait()

                    print("배개 집기")
                    bed_center_x = (bed_box [0]+bed_box[2])/2
                    pillow_center_x = (pillow_box[0]+pillow_box[2])/2

                    print(abs(bed_center_x - pillow_center_x))

                    if abs(bed_center_x - pillow_center_x) < 80:
                        print(abs(bed_center_x - pillow_center_x))
                        current_joints = get_current_posj()  # [j1, j2, j3, j4, j5, j6]
                        print("배개 위치 회전")
                        # 마지막 조인트(j6)만 변경
                        target_joints = current_joints.copy()
                        target_joints[5] += 45  # 예: 30도 회전 (상황에 따라 -도 가능)
                        target_joints[5] += 45 
                        movej(target_joints, vel=50, acc=50)
                        mwait()

                    self.pick_and_place(self.extraction_test[0])
                    movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
                    mwait()
                    break
            elif blanket_box and not pillow_box and not bed_box:
                break
            break

    def get_blanket_angle_and_center(self, frame, x1, y1, x2, y2):
        margin = 0.15
        dx, dy = int((x2 - x1) * margin), int((y2 - y1) * margin)
        x1e, y1e = max(x1 - dx, 0), max(y1 - dy, 0)
        x2e, y2e = min(x2 + dx, frame.shape[1] - 1), min(y2 + dy, frame.shape[0] - 1)
        roi = frame[y1e:y2e, x1e:x2e]
        masked = self.mask_white_area_hsv(roi)
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > 300]
        if not filtered:
            return (x1 + x2) // 2, (y1 + y2) // 2, 0
        all_pts = np.vstack(filtered)
        rect = cv2.minAreaRect(all_pts)
        (cx, cy), (w, h), angle = rect
        if w < h:
            angle += 90
        return cx + x1e, cy + y1e, angle
    
    def robot_control(self):
        movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
        print("1")
        self.run_yolo_control()



# ================= main =================
def main():
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
