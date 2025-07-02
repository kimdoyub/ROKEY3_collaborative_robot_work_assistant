# run_cleanup_service.py
import rclpy
from rclpy.node import Node
from od_msg.srv import Srvchat
from std_msgs.msg import Bool

class RunCleanupService(Node):
    def __init__(self):
        super().__init__('run_cleanup_service')
        self.srv = self.create_service(Srvchat, 'robot_test', self.handle_run_cleanup)
        self.get_logger().info('[robot_test] 서비스 대기 중...')

    def handle_run_cleanup(self, request, response):
        self.get_logger().info(f"✅ [cleanup] 요청 수신: {request.result}")
        print(request.result)
        for i in range(10):
            print(i)
        print('동작 완료')

        response.success = True
        response.feedback = "로봇 움직임 완료"
        print('response')
        return response

def main():
    rclpy.init()
    node = RunCleanupService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
