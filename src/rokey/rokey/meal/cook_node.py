# run_cleanup_service.py
import rclpy
from rclpy.node import Node
from od_msg.srv import Srvchat
from std_msgs.msg import Bool
import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init  # DSR 로봇 초기화 관련

import simpleaudio as sa

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
class RobotTest(Node):
    def __init__(self):
        super().__init__('robot_test')
        self.srv = self.create_service(Srvchat, '/robot_test', self.handle_run_cleanup)
        self.get_logger().info('[robot_test] 서비스 대기 중...')
        self.get_position_request = SrvDepthPosition.Request()
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")

        ########################################################################
        self.file_path_put_obj = "/home/rokey/Desktop/NoObjPutObj.wav"
        self.file_path_put_obj2 = "/home/rokey/Desktop/NoObjTurnOff.wav"

        self.wave_obj = sa.WaveObject.from_wave_file(self.file_path_put_obj)
        self.wave_obj2 = sa.WaveObject.from_wave_file(self.file_path_put_obj2)
        ########################################################################

    ########################################################################################
    ########로봇동작함수
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T
    
    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)  # gripper->camera 변환 행렬 로드
        coord = np.append(np.array(camera_coords), 1)  # 동차 좌표로 변환

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam  # 전체 변환: base -> camera
        td_coord = np.dot(base2cam, coord)  # 최종 좌표 변환

        return td_coord[:3]
    def grip_with_detect_error(self):
        ########################################################################
        for i in range(3):
            gripper.close_gripper()
            while gripper.get_status()[0]:
                time.sleep(0.5)
            if gripper.get_status()[1]:
                break
            else:
                gripper.open_gripper()
                while gripper.get_status()[0]:
                    if i == 2:
                        play_obj2 = self.wave_obj2.play()
                        play_obj2.wait_done()  # 재생이 끝날 때까지 대기
                        self.init_robot()
                        sys.exit()
                    play_obj = self.wave_obj.play()
                    play_obj.wait_done()  # 재생이 끝날 때까지 대기
                    time.sleep(3)
        return
        ########################################################################

    def show_bean(self):
        JReady_bean = posj(0, -20, 130, 0, 20, 90)
        movej(JReady_bean, vel=VELOCITY, acc=ACC)

    def pick_and_place_bean(self, target_name, target_pos):

        if target_name=='bitter': # -y
            self.target_pos = target_pos
            current_pos = get_current_posx()[0]
            pos_y = [current_pos[0], self.target_pos[1], *current_pos[2:]]
            movel(pos_y, vel=VELOCITY, acc=ACC)
            self.target_pos = target_pos
            self.target_pos[0] += 40
            self.target_pos[1] -= 5
            self.target_pos[2] -= 20
            movel(self.target_pos, vel=VELOCITY, acc=ACC)

        elif target_name=='sweet':
            self.target_pos = target_pos
            current_pos = get_current_posx()[0]
            pos_y = [current_pos[0], self.target_pos[1], *current_pos[2:]]
            movel(pos_y, vel=VELOCITY, acc=ACC)
            self.target_pos = target_pos
            self.target_pos[0] += 40
            self.target_pos[1] -= 5
            self.target_pos[2] -= 20
            movel(self.target_pos, vel=VELOCITY, acc=ACC)

        elif target_name=='caramel':
            self.target_pos = target_pos
            current_pos = get_current_posx()[0]
            pos_y = [current_pos[0], self.target_pos[1], *current_pos[2:]]
            movel(pos_y, vel=VELOCITY, acc=ACC)
            self.target_pos = target_pos
            self.target_pos[0] += 40
            self.target_pos[1] -= 10
            self.target_pos[2] -= 20

            movel(self.target_pos, vel=VELOCITY, acc=ACC)

        else:
            print('error')
            return

        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        self.grip_with_detect_error()
        # gripper.close_gripper()
        # while gripper.get_status()[0]:
        #     time.sleep(0.5)

        self.show_bean()
        movel(posx(532.0,-71.94,297.55,72.85,-132.56,-93.62), vel=20, acc=ACC)
        movel(posx(540.90,-85.24,238.63,112.60,167.51,-49.5), vel=VELOCITY, acc=ACC)
        mwait(3)  
        self.show_bean()
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        mwait()  

        gripper.open_gripper()
        move_back = self.target_pos.copy()
        move_back[0] -= 70
        movel(move_back, vel=VELOCITY, acc=ACC)
        while gripper.get_status()[0]:
            time.sleep(0.5)
        # 커피 세팅

    def pick_and_place_cup(self):
        # 컵 옮기기
        movej(posj(37.61, 10.65, 84.71, -0.05, 84.61, 37.58), vel=VELOCITY, acc=ACC) # 커피 제조 장소 # 좌표 따기
        movel(posx(345.72, 275.72, 44.08, 69.81, 179.96, 69.63), vel=VELOCITY, acc=ACC) ########################### 컵 잡는 위치
        self.grip_with_detect_error()

        movej(posj(-21.8, 30.31, 57.42, -0.09, 92.28, -21.85),vel=VELOCITY, acc=ACC) # 두는 곳 위
        movel(posx(536.07, -206.49, 45.0, 72.51, 179.97, 72.33), vel=VELOCITY, acc=ACC) # 내려 놓기

        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def pick_and_place_filter(self):
        movej(posj(-28.94, 8.81, 113.97, -0.66, -32.78, 0.51), vel=VELOCITY, acc=ACC)
        # gripper.close_gripper()
        # while gripper.get_status()[0]:
        #     time.sleep(0.5)
        self.grip_with_detect_error()

        movel(posx(642.32, -343.08, 375.0, 151.58, -90, 180), vel=VELOCITY, acc=ACC) # 필터 잡고 위로 
        movel(posx(576.13, -157.71, 378.82, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movel(posx(545.13, -137.5, 190, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

        movel(posx(576.13, -111.77, 187.24, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)

    def pick_and_place_kettle(self):
        JReady = [0, 0, 90, 0, 90, 0]

        movej(JReady,vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

        movel(posx(757.81, -407.25, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 303.65, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        # gripper.close_gripper()
        # while gripper.get_status()[0]:
        #     time.sleep(0.5)
        self.grip_with_detect_error()

        mwait()
        movel(posx(499.37, -89.98, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        kettleabovefilterJ=posj(23.8, 0, 87.25, -55.69, 83.07, -9.97)
        kettledrippingon1L=posx(485.02, -42.82, 250.00, 124.65, -150.51, 94.51)
        kettledrippingon2L=posx(445.87, -92.49, 250.00, 124.65, -150.51, 94.51)
        kettledrippingon3L=posx(500.53, -125.00, 250.00, 124.65, -150.51, 94.51)
        movej(kettleabovefilterJ,vel=VELOCITY, acc=ACC)
        movel(kettledrippingon1L,vel=VELOCITY, acc=ACC)
        movec(kettledrippingon2L,kettledrippingon3L,vel=VELOCITY, acc=ACC, angle=1080) # 수정 필요
        movel(posx(499.37, -89.98, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 303.65, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

    def pick_and_place_remove_filter(self):

        movel(posx(757.81, -407.25, 303.65, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(757.81, -407.25, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(499.37, -89.98, 366.32, 124.17, -123.61, 88.04), vel=VELOCITY, acc=ACC)
        movel(posx(576.13, -111.77, 187.24, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movel(posx(545.13, -137.5, 190, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC) 
        # gripper.close_gripper()
        # while gripper.get_status()[0]:
        #     time.sleep(0.5)
        self.grip_with_detect_error()


        movel(posx(576.13, -157.71, 378.82, 151.25, -90, -179.5), vel=VELOCITY, acc=ACC)
        movel(posx(642.32, -343.08, 375.0, 151.58, -90, 180), vel=VELOCITY, acc=ACC) 
        movel(posx(646.99, -333.08, 352.71, 152.17, -88.88, -179.75), vel=VELOCITY, acc=ACC) 
        # movej(posj(-28.94, 8.81, 113.97, -0.66, -32.78, 0.51), vel=VELOCITY, acc=ACC) # movel로 따기
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

        current_pos = get_current_posx()[0]
        current_pos[1] += 30
        current_pos[2] += 30
        movel(current_pos, vel=VELOCITY, acc=ACC)
        current_pos = get_current_posx()[0]
        current_pos[1] += 150
        current_pos[2] += 30
        movel(current_pos, vel=VELOCITY,acc=ACC)
    
    ###############시리얼 동작 #######################
    def init_robot_bowl(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_bowl(self):
        JReady = [0, 0, 90, 0, 90, 0] # bo

        movel(posx(487.96,251.26,195.40,98.06,179.97,97.91), vel=VELOCITY, acc=ACC) 
        movel(posx(520.26, 240.05, 50.00, 98.06,179.97,97.91), vel=VELOCITY, acc=ACC) ######################## 볼 잡는 곳

        # gripper.close_gripper() # 잡기
        # # 그리퍼 닫힐 때까지 대기
        # while gripper.get_status()[0]:
        #     time.sleep(0.5)
        self.grip_with_detect_error()

        mwait()
        
        movel(posx(487.96,251.26,195.40,98.06,179.97,97.91), vel=VELOCITY, acc=ACC) # bowl 있는 곳 위로 이동
        movel(posx(233.19,-213.16,195.40,120.71,179.97,120.56), vel=VELOCITY, acc=ACC) # bowl 있는 곳 위로 이동
        movel(posx(233.19,-213.16,55.00,120.71,179.97,120.56), vel=VELOCITY, acc=ACC) # bowl 있는 곳 위로 이동
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

        movej(JReady, vel=VELOCITY, acc=ACC) # 잡은 후 위로 이동
    
    def init_robot_cereal(self):
        JReady_cereal = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11] # Cereal 인식 위치
        movej(JReady_cereal, vel=VELOCITY, acc=ACC)
        current_pos = get_current_posx()[0]
        current_pos[1] += 80
        movel(current_pos, vel=VELOCITY,acc=ACC)
        gripper.open_gripper()
        mwait()
    
    def pick_and_place_milk(self, target_pos):        
        gripper.open_gripper()
        mwait()


        self.target_pos = target_pos
        current_pos = get_current_posx()[0]
        pos_x = [self.target_pos[0], *current_pos[1:]]
        movel(pos_x, vel=VELOCITY, acc=ACC)
        self.target_pos = target_pos
        self.target_pos[1] -= 65
        movel(self.target_pos, vel=VELOCITY, acc=ACC)


        gripper.move_gripper(700)
        while gripper.get_status()[0]:
            time.sleep(0.5)
        current_pos = get_current_posx()[0]
        current_pos[1] += 150
        current_pos[2] += 30
        movel(current_pos, vel=VELOCITY,acc=ACC)
        movel(posx(299.18, -120.39, 282.67, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 122.47, 90.25, -132.52, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 122.47, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 282.67, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        self.target_pos[2] += 25

        movel(self.target_pos,vel=VELOCITY,acc=ACC) # cereal로 이동
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        self.target_pos[1] += 100
        movel(self.target_pos,vel=VELOCITY,acc=ACC) # cereal로 이동
        self.init_robot()
    ######################################################################################

    def init_robot_milk(self):
        JReady_cereal = [12.95, 19.23, 107.16, 97.72, -100.42, 37.11] # Cereal 인식 위치
        movej(JReady_cereal, vel=VELOCITY, acc=ACC)
        current_pos = get_current_posx()[0]
        current_pos[1] += 80
        movel(current_pos, vel=VELOCITY,acc=ACC)
        gripper.open_gripper()
        mwait()
    def get_coffee_pos(self, target, callback=None):
        self.get_logger().info(f"📡 Depth 서비스 요청: {target}")
        request = self.get_position_request
        request.target = target
        future = self.get_position_client.call_async(request)

        def done_callback(future):
            i = 0
            try:
                result = future.result().depth_position.tolist()
                self.get_logger().info(f"📥 위치 수신: {result}")
          
                if sum(result) == 0:
                    self.get_logger().warn("❌ 위치 없음")
                    if callback: callback(None)
                    return

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = get_current_posx()[0]
                print('robot_posx: ',robot_posx)
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)
                print('td_coord: ',td_coord)
                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], MIN_DEPTH)

                target_pos = list(td_coord[:3]) + robot_posx[3:]
                self.get_logger().info(f"🎯 이동 좌표: {target_pos}")

                # ✅ 로봇 동작 수행
                print('커피위치 이동')
                self.pick_and_place_bean(target,target_pos)
                print('주전자 위치 이동')
                self.pick_and_place_kettle() # 커피 뜨거운 물 붓기
                print('필터 제거')
                self.pick_and_place_remove_filter() # 필터 제거하기
                self.init_robot()
                # ✅ 결과 후속 처리 (옵션 콜백)
                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ depth 서비스 응답 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)
        return True

    def pick_and_place_cereal(self, target_name, target_pos):        
        gripper.open_gripper()
        mwait()

        if target_name=='frosed': # -y
            self.target_pos = target_pos
            current_pos = get_current_posx()[0]
            pos_x = [self.target_pos[0], *current_pos[1:]]
            movel(pos_x, vel=VELOCITY, acc=ACC)
            self.target_pos = target_pos
            self.target_pos[1] -= 65
            movel(self.target_pos, vel=VELOCITY, acc=ACC)

        elif target_name=='choco':
            self.target_pos = target_pos
            current_pos = get_current_posx()[0]
            pos_x = [self.target_pos[0], *current_pos[1:]]
            movel(pos_x, vel=VELOCITY, acc=ACC)
            self.target_pos = target_pos
            self.target_pos[1] -= 65
            movel(self.target_pos, vel=VELOCITY, acc=ACC)

        else:
            print('error')
            return

        gripper.move_gripper(730)
        while gripper.get_status()[0]:
            time.sleep(0.5)
        current_pos = get_current_posx()[0]
        current_pos[1] += 100
        movel(current_pos, vel=VELOCITY,acc=ACC)
        movel(posx(299.18, -120.39, 282.67, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 122.47, 90.25, -132.52, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 122.47, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        movel(posx(299.18, -120.39, 282.67, 90.25, -89.76, -89.87), vel=VELOCITY, acc=ACC)
        self.target_pos[2] += 15

        movel(self.target_pos,vel=VELOCITY,acc=ACC) # cereal로 이동
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        self.target_pos[1] += 100
        movel(self.target_pos,vel=VELOCITY,acc=ACC) # cereal로 이동

    def get_cereal_pos(self, target, callback=None):
        self.get_logger().info(f"📡 Depth 서비스 요청: {target}")
        request = self.get_position_request
        request.target = target
        future = self.get_position_client.call_async(request)

        def done_callback(future):
            try:
                result = future.result().depth_position.tolist()
                self.get_logger().info(f"📥 위치 수신: {result}")

                if sum(result) == 0:
                    self.get_logger().warn("❌ 위치 없음")
                    if callback: callback(None)
                    return

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = get_current_posx()[0]
                print('robot_posx: ',robot_posx)
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)
                print('td_coord: ',td_coord)
                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], MIN_DEPTH)

                target_pos = list(td_coord[:3]) + robot_posx[3:]
                self.get_logger().info(f"🎯 이동 좌표: {target_pos}")

                # ✅ 로봇 동작 수행
                self.pick_and_place_cereal(target,target_pos)
                self.init_robot_milk()

                # ✅ 결과 후속 처리 (옵션 콜백)
                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ depth 서비스 응답 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)
        return True

    def get_milk_pos(self, callback=None):
        target = "milk"
        self.get_logger().info(f"📡 Depth 서비스 요청: {target}")
        request = self.get_position_request
        request.target = target
        future = self.get_position_client.call_async(request)

        def done_callback(future):
            try:
                result = future.result().depth_position.tolist()
                self.get_logger().info(f"📥 우유 위치 수신: {result}")

                if sum(result) == 0:
                    self.get_logger().warn("❌ 우유 위치 없음")
                    if callback: callback(None)
                    return

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)
                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], MIN_DEPTH)

                target_pos = list(td_coord[:3]) + robot_posx[3:]
                self.get_logger().info(f"🎯 우유 이동 좌표: {target_pos}")

                # 여기에 우유 pick 동작 삽입
                self.pick_and_place_milk(target_pos)

                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ 우유 depth 서비스 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)

    def coffee_move(self, coffee_taste):
        print(f'커피 {coffee_taste}')
        # home
        self.init_robot()
        self.pick_and_place_cup()
        self.init_robot()
        self.pick_and_place_filter()
        # for i in range(3):
        #     gripper.move_gripper(800)
        #     while gripper.get_status()[0]:
        #         time.sleep(0.5)
        #     if gripper.get_status()[1]:
        #         break
        #     else:
        #         gripper.open_gripper()
        #         while gripper.get_status()[0]:
        #             if i == 2:
        #                 play_obj2 = self.wave_obj2.play()
        #                 play_obj2.wait_done()  # 재생이 끝날 때까지 대기
        #                 self.init_robot()
        #                 sys.exit()
        #             play_obj = self.wave_obj.play()
        #             play_obj.wait_done()  # 재생이 끝날 때까지 대기
        #             time.sleep(3)

        gripper.move_gripper(800)
        while gripper.get_status()[0]:           # self.pick_and_place_bean(target,target_pos)
            # self.pick_and_place_kettle() # 커피 뜨거운 물 붓기
            time.sleep(0.5)
        self.show_bean()
        # 거리측정
        self.get_coffee_pos(coffee_taste)

    def cereal_move(self, cereal_taste):
        print(f'시리얼: {cereal_taste}')
        self.init_robot_bowl() # 그릇을 잡기 위한 준비
        self.pick_and_place_bowl() # 그릇을 시리얼 제조 장소로 이동
        self.init_robot_cereal() # 시리얼을 잡기 위한 초기화
        self.get_cereal_pos(cereal_taste,lambda _: self.get_milk_pos())

    def handle_run_cleanup(self, request, response):
        try:
            print()
            self.get_logger().info(f"✅ [robot_test] 요청 수신: {request.result}")
            text = request.result
            text2 = eval(text)
            print(text2)
            coffee = False
            cereal = False

            coffee = text2[0]
            cereal = text2[1]

            coffee_taste = text2[2]
            cereal_taste = text2[-1]
            #######################################
            ##############로봇무브##################
            #######################################
            # 커피
            if coffee and cereal:
                print(f'커피: {coffee_taste}, 시리얼:{cereal_taste}')
                # home
                self.init_robot()
                self.pick_and_place_cup()
                self.init_robot()
                self.pick_and_place_filter()
                gripper.move_gripper(800)
                while gripper.get_status()[0]:           # self.pick_and_place_bean(target,target_pos)
                    # self.pick_and_place_kettle() # 커피 뜨거운 물 붓기
                    time.sleep(0.5)
                self.show_bean()
                # 거리측정
                self.get_coffee_pos(coffee_taste,lambda _: self.cereal_move(cereal_taste))
                # self.init_robot_bowl() # 그릇을 잡기 위한 준비
                # self.pick_and_place_bowl() # 그릇을 시리얼 제조 장소로 이동
                # self.init_robot_cereal() # 시리얼을 잡기 위한 초기화
                # self.get_cereal_pos(cereal_taste,lambda _: self.get_milk_pos())

            elif coffee:
                self.coffee_move(coffee_taste)
            elif cereal:
                self.cereal_move(cereal_taste)
                

            response.success = True
            response.feedback = "아침조리 완료"
            return response

        except Exception as e:
            print(f"❌ 예외 발생: {e}")
            response.success = False
            response.feedback = f"에러 발생: {e}"
            return response

def main():
    # rclpy.init()
    node = RobotTest()
    print("RobotTest node 시작")
    rclpy.spin(node)
    print("RobotTest node 끝")
    node.destroy_node()
    print("RobotTest node destroy")
    rclpy.shutdown()
    print("rclpy shutdown")
if __name__ == '__main__':
    main()
