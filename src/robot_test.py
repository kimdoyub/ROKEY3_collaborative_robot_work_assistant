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
import DR_init  # DSR 로봇 초기화 관련

# ROS2 서비스 정의
from od_msg.srv import SrvDepthPosition  # 3D 좌표 요청 서비스
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG  # OnRobot 그리퍼 컨트롤 클래스

# 패키지 경로 가져오기
package_path = get_package_share_directory("rokey")

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
        self.init_robot()
        self.srv = self.create_service(Srvchat, '/robot_test', self.handle_run_cleanup)
        self.get_logger().info('[robot_test] 서비스 대기 중...')
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")

        self.get_position_request = SrvDepthPosition.Request()
        self.target_pos = []
    # 위치(x,y,z) + 회전(rx,ry,rz) 정보를 4x4 변환 행렬로 변환
    
    def get_target_pos(self, target, callback=None):
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
                self.move_robot_to(target_pos)

                # ✅ 결과 후속 처리 (옵션 콜백)
                if callback:
                    callback(target_pos)

            except Exception as e:
                self.get_logger().error(f"❌ depth 서비스 응답 실패: {e}")
                if callback: callback(None)

        future.add_done_callback(done_callback)
        return True

    def handle_run_cleanup(self, request, response):
        try:
            self.get_logger().info(f"✅ [robot_test] 요청 수신: {request.result}")
            extraction_test = eval(request.result)  # 문자열 리스트라면 변환 필요
            print(extraction_test)
            if extraction_test[0]:  # 커피
                target = extraction_test[2]
            elif extraction_test[1]:  # 시리얼
                target = extraction_test[-1]
            else:
                response.success = False
                response.feedback = "해당 메뉴 없음"
                return response

            # ✅ 1단계: 로봇이동1
            self.move_robot_pre()

            # ✅ 2단계: get_target_pos → 결과 오면 후속 동작 실행
            def after_get_position(target_pos):
                if not target_pos:
                    self.get_logger().error("❌ 타겟 위치 없음")
                    response.success = False
                    response.feedback = "타겟 위치를 찾지 못했습니다."
                    return

                # ✅ 3단계: 로봇이동2
                self.move_robot_to(target_pos)

                # ✅ 4단계: 서비스 응답 완료
                response.success = True
                response.feedback = "로봇 동작 완료"
                self.get_logger().info("🟢 response 반환 직전")
                # ❗ 여기서는 return response 안 됨 (이미 콜백 구조니까)
                # 대신 status를 publish하거나, 내부 상태를 갱신

            # self.get_target_pos(target, after_get_position)
            self.get_target_pos(target)
            # ❗ 주의: 여기서 return response 하면 안 됨
            # 서비스는 콜백 안에서 마무리돼야 정확히 맞습니다
            # 하지만 rclpy는 callback 안에서 response를 return하는 걸 지원하지 않음 ❌

            return response  # 이 return은 placeholder일 뿐, 실제 응답은 안 돌아감

        except Exception as e:
            print(f"❌ 예외 발생: {e}")
            response.success = False
            response.feedback = f"에러 발생: {e}"
            return response

def main():
    rclpy.init()
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


