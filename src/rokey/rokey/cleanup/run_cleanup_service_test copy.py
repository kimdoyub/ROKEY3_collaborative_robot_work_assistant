import rclpy
from rclpy.node import Node
from od_msg.srv import Srvchat
from std_srvs.srv import Trigger
import time
from std_msgs.msg import Bool
class RunCleanService(Node):
    def __init__(self):
        super().__init__('run_menu_service_test')
        self.srv = self.create_service(Srvchat, '/run_cleanup', self.handle_clean_menu)
        # self.robot_client = self.create_client(Trigger, '/robot_test')
        # while not self.robot_client.wait_for_service(timeout_sec=3.0):
        #     self.get_logger().info('Waiting for robot_test service...')
        self.get_logger().info('🍽️ [run_cleanup] 서비스 대기 중...')
        self.publisher_ = self.create_publisher(Bool, '/llm_activation', 10)

    def call_subject_service(self):
        print("call_subject_service 실행")
        client = self.create_client(Srvchat, '/robot_test')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"robot_test 서비스를 기다리는 중...")

        request = Srvchat.Request()
        request.result = '침구 정리 실행'
        future = client.call_async(request)
        print('침구정리 시작')
        def done_callback(future):
            try:
                res = future.result()
                self.get_logger().info(f"✅ 서비스 완료: {res.feedback}")
                return res.feedback
            except Exception as e:
                self.get_logger().error(f"❌ 응답 처리 중 예외: {e}")

        future.add_done_callback(done_callback)
        return True  # 비동기이므로 바로 True 반환
    
    def handle_clean_menu(self, request, response):
        self.get_logger().info(f"✅ [meal] 요청 수신: {request.result}")
        self.get_logger().info(f"침구정리 시작")
        answer = self.call_subject_service()

        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info("🧍 Jarvis 대기모드 진입")


        if not answer:
            response.success = False
            response.feedback = "하위 동작 실패"
            return response
        # get_keyword
        response.success = True
        response.feedback = "메뉴 동작 완료"

        return response

def main():
    rclpy.init()
    node = RunCleanService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
