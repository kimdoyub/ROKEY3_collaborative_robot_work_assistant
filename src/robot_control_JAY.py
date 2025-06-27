import os
import time
import sys
from scipy.spatial.transform import Rotation # For what?
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init  # DSR 로봇 초기화 관련

# ROS2 서비스 정의
from od_msg.srv import SrvDepthPosition  # 3D 좌표 요청 서비스
from std_srvs.srv import Trigger  # 키워드 인식 서비스

from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG  # OnRobot 그리퍼 컨트롤 클래스

# 패키지 경로 가져오기
package_path = get_package_share_directory("pick_and_place_voice")

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60  # 속도와 가속도
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0     # z축 깊이 보정
MIN_DEPTH = 2.0         # 최소 z값 제한

# 로봇 초기 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

# 로봇 제어 함수 import
try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, movec, move_periodic
    # trans & spiral have errors
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# 그리퍼 객체 생성
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


# 로봇 제어 클래스 정의
class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()  # 로봇 초기 위치로 이동 및 그리퍼 open

        # 3D 좌표를 가져오는 서비스 클라이언트 생성
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")

        self.get_position_request = SrvDepthPosition.Request()

        # 음성 키워드 인식 서비스 클라이언트 생성
        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")

        self.get_keyword_request = Trigger.Request()

    # 위치(x,y,z) + 회전(rx,ry,rz) 정보를 4x4 변환 행렬로 변환
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    # 카메라 좌표계에서 로봇 기준 좌표계로 변환
    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)  # gripper->camera 변환 행렬 로드
        coord = np.append(np.array(camera_coords), 1)  # 동차 좌표로 변환

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam  # 전체 변환: base -> camera
        td_coord = np.dot(base2cam, coord)  # 최종 좌표 변환

        return td_coord[:3]

    # 메인 음성인식 및 pick & place 로직
    def robot_control(self): ##### Main Code #####
        target_list = []
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)

        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()
            target_list = get_keyword_result.message.split()  # 인식된 키워드 분할

            for target in target_list:
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    #####################################################
                    self.init_robot()  # 매 픽앤플레이스 후 초기화
                    self.prepare_brew()
                    self.pick_and_place_target(target_pos)
                    self.brew_coffee()
                    self.remove_filter()
                    self.init_robot()  # 매 픽앤플레이스 후 초기화
                    #####################################################
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    # 지정한 물체의 좌표를 가져와서 로봇 기준으로 변환
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

    # 로봇 초기 위치로 이동 + 그리퍼 열기
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def prepare_brew(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

        # 컵 옮기기
        movej()
        movel()
        gripper.close_gripper()
        mwait()
        movel()

        movej() # 커피 제조 장소
        movel()
        gripper.open_gripper()
        movel()
        movej(JReady, vel=VELOCITY, acc=ACC)

        # Coffee Filter 옮기기
        movej()
        movel()
        gripper.close_gripper()
        mwait()
        movel()

        movej()
        movel()
        gripper.open_gripper()
        movel()
        movej(JReady, vel=VELOCITY, acc=ACC)

    # 주어진 좌표로 이동 → 물체 집기 → 그리퍼 열기
    # 이 단계에서 커피 선택
    def pick_and_place_target(self, target_pos):
        JReady = [0, 0, 90, 0, 90, 0]
        movel(target_pos, vel=VELOCITY, acc=ACC) # 원두 있는 곳으로 이동
        mwait()
        gripper.close_gripper() # 잡기

        # 그리퍼 닫힐 때까지 대기
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()
        movej(JReady, vel=VELOCITY, acc=ACC)
        movej() # brew area 이동
        movel() # 붓기 (회전)
        movel() # brew area 이동
        movej() # 원위치 윗 부분 이동
        movel() # 내려 놓을 위치 이동
        gripper.open_gripper() # 내려놓기

        while gripper.get_status()[0]:
            time.sleep(0.5)

    def brew_coffee(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
        
        # kettle
        movej()
        movel()
        gripper.close_gripper()
        mwait()
        movel()

        # coffee brew
        movej()
        movec()
        move_periodic()
        movel()

        # returning
        movej()
        movel()
        gripper.open_gripper()
        mwait()

    def remove_filter(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

        # filter 제거
        movej()
        movel()
        gripper.close_gripper() # 필터잡기
        mwait()
        movel()

        movej() # 개수대 이동
        movel()
        gripper.open_gripper() # 필터 놓기
        movel()
        movel()
        gripper.open_gripper() # 필터 놓기
        movel()
        movej(JReady, vel=VELOCITY, acc=ACC)



# 메인 함수
def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
