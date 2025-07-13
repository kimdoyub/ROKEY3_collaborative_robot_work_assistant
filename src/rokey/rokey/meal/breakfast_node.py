# run_menu_service.py
import rclpy
from rclpy.node import Node
from od_msg.srv import SrvDepthPosition
from od_msg.srv import Srvchat
import time
from std_srvs.srv import Trigger
# from rokey.meal.detection_mod1 import ObjectDetector
class RunMenuService(Node):
    def __init__(self):
        super().__init__('run_menu_service_test')
        self.srv = self.create_service(Srvchat, '/run_menu', self.handle_run_menu)
        self.get_logger().info('🍽️ [run_menu] 서비스 대기 중...')


    def call_subject_service(self, result):
        print("call_subject_service 실행")
        client = self.create_client(Srvchat, '/robot_test')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"robot_test 서비스를 기다리는 중...")

        request = Srvchat.Request()
        request.result = result
        future = client.call_async(request)

        def done_callback(future):
            try:
                res = future.result()
                self.get_logger().info(f"✅ 서비스 완료: {res.feedback}")
                return res.feedback
            except Exception as e:
                self.get_logger().error(f"❌ 응답 처리 중 예외: {e}")

        done1 = future.add_done_callback(done_callback)
        print(done1)
        return True  # 비동기이므로 바로 True 반환



    def handle_run_menu(self, request, response):
        self.get_logger().info(f"✅ [meal] 요청 수신: {request.result}")
        text = request.result
        try:
            object, target = text.strip().split("/")
        except ValueError:
            response.success = False
            response.feedback = "형식이 올바르지 않습니다. 예: '커피 / 쓴맛'"
            return response
        object, target = text.strip().split("/")
        menu_list = []
        tool_dict = {
            '쓴맛': "bitter",
            '카라멜': "caramel", 
            '초코': "choco",
            '콘프로스트': "frosed",
            '단맛': "sweet"
            }

        coffee = False
        cereal = False
        if '커피' in object:
            coffee = True
        else:
            coffee = False
        if '시리얼' in object:
            cereal = True
        else:
            cereal = False
        
        menu_list.append(coffee)
        menu_list.append(cereal)

        if coffee == True and cereal == False:
            menu_list.append(tool_dict[target.strip().split(' ')[0]])
            menu_list.append(None)
        elif coffee == True and cereal == True:
            menu_list.append(tool_dict[target.strip().split(' ')[0]])
            menu_list.append(tool_dict[target.strip().split(' ')[1]])
        elif coffee == False and cereal == True:
            menu_list.append(None)
            menu_list.append(tool_dict[target.strip()])
        else:
            pass
        print(menu_list)
        print('-동작대기-')
        answer = self.call_subject_service(str(menu_list))
        print('-동작완료-')
        print('응답: ', answer)
        time.sleep(15)
        if not answer:
            response.success = False
            response.feedback = "하위 동작 실패"
            return response
        # get_keyword
        print(menu_list)
        response.success = True
        response.feedback = "메뉴 동작 완료"
        return response

def main():
    rclpy.init()
    node = RunMenuService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
