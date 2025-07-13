# run_cleanup_service.py
import rclpy
from rclpy.node import Node
from od_msg.srv import Srvchat
from std_msgs.msg import Bool

class RobotTest(Node):
    def __init__(self):
        super().__init__('robot_test')
        self.srv = self.create_service(Srvchat, '/robot_test', self.handle_run_cleanup)
        self.get_logger().info('[robot_test] 서비스 대기 중...')

    def handle_run_cleanup(self, request, response):
        try:
            print()
            self.get_logger().info(f"✅ [robot_test] 요청 수신: {request.result}")
            print(request.result)

            for i in range(10):
                print(i)

            response.success = True
            response.feedback = "로봇 움직임 완료"
            print("🟢 response 반환 직전")
            return response

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
