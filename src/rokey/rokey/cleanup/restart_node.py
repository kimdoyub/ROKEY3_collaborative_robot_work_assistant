import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
from ultralytics import YOLO
import time

class YoloPeopleDetector(Node):
    def __init__(self):
        super().__init__('yolo_people_detector')
        self.publisher_ = self.create_publisher(Bool, '/llm_activation', 10)
        self.model = YOLO('yolov8n.pt')  # 기본 lightweight 모델
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("❌ 웹캠을 열 수 없습니다.")
            exit(1)

        self.get_logger().info("🚀 YOLO 사람 감지 시작")

    def run(self):
        try:
            while rclpy.ok():
                start_time = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("⚠️ 프레임 수신 실패")
                    continue

                results = self.model(frame, verbose=False)[0]
                detected = False

                for box in results.boxes:
                    cls_id = int(box.cls[0])
                    if cls_id == 0:  # 'person'
                        detected = True
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, "person", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if detected:
                    msg = Bool()
                    msg.data = True
                    self.publisher_.publish(msg)
                    self.get_logger().info("🧍 사람 감지됨 → /llm_activation = True")

                # 화면 출력
                cv2.imshow("YOLO CCTV", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                elapsed = time.time() - start_time
                sleep_time = max(0.03 - elapsed, 0)  # 30fps 유지용 sleep
                time.sleep(sleep_time)

        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = YoloPeopleDetector()
    node.run()

if __name__ == '__main__':
    main()
