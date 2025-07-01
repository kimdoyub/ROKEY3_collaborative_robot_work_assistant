import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init  # DSR 로봇 초기화 관련

# ROS2 서비스 정의
from od_msg.srv import SrvDepthPosition  # 3D 좌표 요청 서비스

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
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# 그리퍼 객체 생성
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


# 로봇 제어 클래스 정의
class RobotController(Node):
    def __init__(self):
        super().__init__("motion_test_node")
        self.init_robot()  # 로봇 초기 위치로 이동 및 그리퍼 open

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")

        self.get_position_request = SrvDepthPosition.Request()
        self.extraction_test = [True, True, 'bitter', 'choco']
        self.target_pos = []

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

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    # 메인 음성인식 및 pick & place 로직
    def robot_control(self): ##### Main Code #####

        #####################################################
        ################# Coffee Preparation ################
        #####################################################
        self.init_robot_cup()  # 매 픽앤플레이스 후 초기화
        self.pick_and_place_cup()
        self.init_robot()
        self.pick_and_place_filter()
        if self.extraction_test[0]:
            self.coffee_flavor = self.extraction_test[2]
        else:
            self.get_logger().info('Coffee Keyword not received')
        self.pick_and_place_bean(self.coffee_flavor)
        self.pick_and_place_kettle()



        # self.pick_and_place_remove_filter()
        # self.init_robot()  # 매 픽앤플레이스 후 초기화

        # #####################################################
        # ################# Cereal Preparation ################
        # #####################################################
        # # init 위치 맞춰줄 것
        # self.init_robot_bowl()
        # self.pick_and_place_cup()
        # self.init_robot_cereal()
        # self.pick_and_place_cereal()
        # self.init_robot_milk()
        # self.pick_and_place_milk()

    ################################################################
    ################# Coffee Preparation Definition ################
    #####################################################3##########

    def init_robot_cup(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_cup(self):
        # JReady = [0, 0, 90, 0, 90, 0]
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # gripper.open_gripper()
        # mwait()

        # 컵 옮기기
        movej(posj(37.61, 10.65, 84.71, -0.05, 84.61, 37.58), vel=VELOCITY, acc=ACC) # 커피 제조 장소 # 좌표 따기
        movel(posx(345.72, 275.72, 44.08, 69.81, 179.96, 69.63), vel=VELOCITY, acc=ACC) # 내려가기  ############## force control로 바꾸기
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        ##########################
        movej(posj(-21.8, 30.31, 57.42, -0.09, 92.28, -21.85),vel=VELOCITY, acc=ACC) # 두는 곳 위
        movel(posx(536.07, -206.49, 45.0, 72.51, 179.97, 72.33), vel=VELOCITY, acc=ACC) # 내려 놓기
        # movel(posx(576.13, -111.77, 47.5, 72.51, 179.97, 72.33), vel=VELOCITY, acc=ACC) # 내려 놓기
        # movel(posx(576.13, -157.71, 47.5, 72.51, 179.97, 72.33), vel=VELOCITY, acc=ACC) # 내려 놓기

        # movel(posx(576.13, -111.77, 45.0, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        # movel(posx(576.13, -157.71, 187.23, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)


        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        #################################
        #     gripper.close_gripper()
        #     while gripper.get_status()[0]:
        #         time.sleep(0.5)
        #     mwait()
        #     mwait()


        #     movej(posj(37.61, 10.65, 84.71, -0.05, 84.61, 37.58), vel=VELOCITY, acc=ACC) # 커피 제조 장소 # 좌표 따기
        #     movel(posx(345.72, 275.72, 44.08, 69.81, 179.96, 69.63), vel=VELOCITY, acc=ACC) # 내려가기  ############## force control로 바꾸기
        #     gripper.open_gripper()
        #     while gripper.get_status()[0]:
        #         time.sleep(0.5)
        #     movel(posx(345.71, 275.71, 148.61, 72.27, 179.97, 72.09), vel=VELOCITY, acc=ACC) # 커피 제조 장소 # 위에 거 이용
        #     # movej(posj(19.69, 15.77, 71.8, -0.06, 92.41, 19.66), vel=VELOCITY, acc=ACC)

    def pick_and_place_filter(self):
        movej(posj(-28.94, 8.81, 113.97, -0.66, -32.78, 0.51), vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        movel(posx(642.32, -343.08, 375.0, 151.58, -90, 180), vel=VELOCITY, acc=ACC) # 필터 잡고 위로 
        movel(posx(576.13, -157.71, 378.82, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movel(posx(545.13, -137.5, 190, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        movel(posx(576.13, -157.71, 378.82, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)

        movel(posx(576.13, -111.77, 187.24, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)

    def pick_and_place_bean(self, target_name):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        JReady_bean = posj(0, -20, 130, 0, 20, 90)
        movej(JReady_bean, vel=VELOCITY, acc=ACC)
        if target_name:
            self.target_pos = self.get_target_pos(target_name)
            self.target_pos[0] += 40
            self.target_pos[2] -= 20
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        movej(JReady_bean, vel=VELOCITY, acc=ACC)
        # movel(posx(515.02, -92.82, 286.32, -124.65, 150.51, -94.51), vel=VELOCITY, acc=ACC)   
        movel(posx(532.0,-71.94,297.55,72.85,-132.56,-93.62), vel=20, acc=ACC)
        movel(posx(540.90,-85.24,238.63,112.60,167.51,-49.5), vel=VELOCITY, acc=ACC)
        mwait(3)  
        movej(JReady_bean, vel=VELOCITY, acc=ACC)
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def pick_and_place_kettle(self):
        JReady = [0, 0, 90, 0, 90, 0]

        movej(JReady,vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

        movel(posx(757.81, -407.25, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 303.65, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()
        movel(posx(499.37, -89.98, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        kettleabovefilterJ=posj(23.8, 0, 87.25, -55.69, 83.07, -9.97)
        kettledrippingon1J=posj(6.02, 7.69, 73.89, -25.85, 84, -24.53)
        kettledrippingon1L=posx(515.02, -92.82, 286.32, 124.65, -150.51, 94.51)
        kettledrippingon2J=posj(-0.89, 1.69, 80.45, -24.04, 80.61, -30.55)
        kettledrippingon2L=posx(475.87, -142.49, 286.32, 124.65, -150.51, 94.51)
        kettledrippingon3J=posj(1.27, 14.7, 65.34, -24.57, 83.39, -29.47)
        kettledrippingon3L=posx(565.53, -126.42, 286.32, 124.65, -150.51, 94.51)
        movej(kettleabovefilterJ,vel=VELOCITY, acc=ACC)
        movel(kettledrippingon1L,vel=VELOCITY, acc=ACC)
        movec(kettledrippingon2L,kettledrippingon3L,vel=VELOCITY, acc=ACC, angle=1080)
        movel(posx(499.37, -89.98, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 303.65, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

    def pick_and_place_remove_filter(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

        # filter 제거
        movej() # 필터 근처
        movel() # 접근
        gripper.close_gripper() # 필터잡기
        mwait()
        movel() # 후퇴

        movej() # 개수대 이동
        movel() # 접근
        gripper.open_gripper() # 필터 놓기
        movel() # 후퇴
        movej(JReady, vel=VELOCITY, acc=ACC)

    ################################################################
    ################# Cereal Preparation Definition ################
    ################################################################
            # JReady_cereal = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11] # Cereal 인식 위치

    def init_robot_bowl(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
    
    def pick_and_place_bowl(self):
        JReady = [0, 0, 90, 0, 90, 0] # bo

        movel(vel=VELOCITY, acc=ACC) # bowl 있는 곳으로 이동 # x,y,z 보정 예정
        mwait()
        gripper.close_gripper() # 잡기
        # 그리퍼 닫힐 때까지 대기
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        movej(JReady, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
        movej() # bowl 두는 곳으로 이동
        movel() # 내려가기
        gripper.open_gripper() # 내려놓기 # force로도 처리 가능
        while gripper.get_status()[0]:
            time.sleep(0.5)
        movej(JReady, vel=VELOCITY, acc=ACC) # 잡은 후 위로이동 
    
    def init_robot_cereal(self):
        JReady_cereal = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11] # Cereal 인식 위치
        movej(JReady_cereal, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_cereal(self):
        JReady_cereal = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11]
        movel(vel=VELOCITY, acc=ACC) # cereal로 이동
        mwait()
        gripper.close_gripper() # 잡기

        # 그리퍼 닫힐 때까지 대기
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        movej(JReady_cereal, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
        movej() # bowl 위로 이동
        movel() # 내려가기
        move_periodic() # 시리얼 회전으로 붓기 # 수정
        movel() # 올라가기
        movej(JReady_cereal, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
        movej() # 시리얼 둘 위치로 이동
        movel() # 정밀하게 이동

        gripper.open_gripper() # 내려놓기 # force로도 처리 가능
        while gripper.get_status()[0]:
            time.sleep(0.5)

        movel() # 정밀하게 후퇴

    def init_robot_milk(self):
        JReady_milk = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11] # Milk 인식 위치
        movej(JReady_milk, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_milk(self):
        JReady_milk = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11]
        movel(vel=VELOCITY, acc=ACC) # milk로 이동
        mwait()
        
        gripper.close_gripper() # 잡기
        # 그리퍼 닫힐 때까지 대기
        while gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        movej(JReady_milk, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
        movej() # bowl 위로 이동
        movel() # 내려가기
        move_periodic() # milk 회전으로 붓기
        movel() # 올라가기
        movej(JReady_milk, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
        movej() # milk 둘 위치로 이동
        movel() # 정밀하게 이동

        gripper.open_gripper() # 내려놓기 # force로도 처리 가능
        while gripper.get_status()[0]:
            time.sleep(0.5)

        movel() # 정밀하게 후퇴


# 메인 함수
def main(args=None):
    node = RobotController()
    if rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
