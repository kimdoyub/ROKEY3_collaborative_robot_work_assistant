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
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
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
ANGLE_THRESHOLD = 45.0
# 로봇 초기 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

# 로봇 제어 함수 import
try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, movec, move_periodic,get_current_posj
    # trans & spiral have errors
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# 그리퍼 객체 생성
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
BED_POSITION = [366.91, 88.86, 181.33, 41.85, -179.84, 41.94]
class RobotTestService(Node):
    def __init__(self):
        super().__init__('robot_test_service')
        self.srv = self.create_service(Srvchat, '/clean_robot_test', self.handle_robot_test)
            # 서비스 클라이언트 세팅 직후 (__init__에 추가)

        self.create_subscription(
            Float32,
            '/bed_angle',
            self.angle_callback,
            10
        )
        self.get_logger().info('🤖 [robot_test] 서비스 서버 대기 중...')

        self.get_position_request = SrvDepthPosition.Request()
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")

    #########################################################
    ################로봇 동작 ################################
    def init_robot(self):
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def angle_callback(self, msg: Float32):
        # YoloImageNode에서 퍼블리시된 침대(이불) 각도를 저장
        self.bed_angle = msg.data

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
    
    def pat_motion(self, pos):
        ##########################토닥이기 위 아래 좌표 잡기##################
        top = [pos[0], pos[1]-40, pos[2]-80, *pos[3:]]
        bottom = [pos[0], pos[1]-40, pos[2] -135, *pos[3:]]
        movel(top, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()
        mwait()
        for _ in range(3):
            movel(bottom, vel=VELOCITY, acc=ACC)
            mwait()
            movel(top, vel=VELOCITY, acc=ACC)
            mwait()
            top[1] += 30
            bottom[1] += 30

    ######################################################

    #########################################################
    #@@@@@@@@@@@@@@@@@@@@@@로봇 동작 @@@@@@@@@@@@@@@@@@@@@@@@#
    def get_blanket_pos(self, target, callback=None):
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
                self.pick_and_place(target_pos)
                self.pat_motion(BED_POSITION)
                self.init_robot()
                # ✅ 결과 후속 처리 (옵션 콜백)
                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ depth 서비스 응답 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)
        return True

    def get_pillow_pos(self, callback=None):
        target = 'pillow'
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
                self.pick_and_place_pillow(target_pos)
                # self.pat_motion(BED_POSITION)
                # self.init_robot()
                # ✅ 결과 후속 처리 (옵션 콜백)
                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ depth 서비스 응답 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)
        return True

    def pick_and_place(self, target_pos):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        
        if target_pos:
            self.target_pos = target_pos

        approach_pos = self.target_pos.copy()
        approach_pos[2] += 120  # z축 위로 접근
        angle = self.bed_angle or 0.0
        print(f'이불 각도: {angle}')
        # gripper 회전
        if angle < 90 - ANGLE_THRESHOLD or angle > 90 + ANGLE_THRESHOLD:
            cj = get_current_posj()
            tj = cj.copy()
            tj[5] += 90
            movej(tj, vel=50, acc=50)
            mwait()
            current_pose = get_current_posx()[0]   # [x,y,z,rx,ry,rz]
            self.target_pos[3:] = current_pose[3:] 
            
        approach_pos = self.target_pos.copy()
        approach_pos[2] += 120
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
        let = [BED_POSITION[0], BED_POSITION[1]+5, BED_POSITION[2]-120, *BED_POSITION[3:]]           
        movel(let, vel=20, acc=ACC)
        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def pick_and_place_pillow(self, target_pos):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

        if target_pos:
            self.target_pos = target_pos

        approach_pos = self.target_pos.copy()
        approach_pos[2] += 120  # z축 위로 접근

        movel(approach_pos, vel=VELOCITY, acc=ACC)
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        put_pillow = self.target_pos.copy()
        put_pillow[1] -= 10
        movel(put_pillow, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        self.target_pos[2]+=120
        movel(self.target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        temp_leave = BED_POSITION.copy()
        temp_leave[1] -= 50
        movel(temp_leave, vel=VELOCITY, acc=ACC)
        # movel(posx(515.02, -92.82, 286.32, -124.65, 150.51, -94.51), vel=VELOCITY, acc=ACC)        
        let = [temp_leave[0], temp_leave[1], temp_leave[2]-120, *temp_leave[3:]]       
  
        movel(let, vel=20, acc=ACC)
        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
    
    #########################################################

    def handle_robot_test(self, request, response):
        # 초기화
        self.init_robot()
        # depth 서비스 콜 전에 bed_angle이 들어올 때까지 잠시 대기
        start = time.time()
        while self.bed_angle is None and time.time() - start < 2.0:
            rclpy.spin_once(self)
        # get_blanket_pos → pick → pat → init 로직 계속
        self.get_blanket_pos('blanket', lambda pos: self._after_blanket(pos))
        response.success = True
        response.feedback = "로봇 동작 완료"
        return response
    
    def _after_blanket(self, pos):
        # blanket 위치 계산 후 (여기까지 오면 bed_angle 사용 가능)
        # 그 다음 베개 픽앤플레이스
        self.get_pillow_pos()

    # … 나머지 메서드 …

def main():
    node = RobotTestService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()