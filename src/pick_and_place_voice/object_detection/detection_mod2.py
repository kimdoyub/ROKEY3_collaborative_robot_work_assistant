import numpy as np
import rclpy
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel


class ObjectDetector:
    def __init__(self):
        self.img_node = ImgNode()  # 내부적으로 Node 역할을 하지만 따로 spin하지 않음
        self.model = YoloModel()
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        print("✅ ObjectDetector initialized.")

    def get_coords(self, target):
        """지정된 타겟 객체의 카메라 좌표를 반환합니다."""
        print(f"[INFO] target: {target}")
        coords = self._compute_position(target)
        return [float(x) for x in coords]

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            print("[WARN] No detection found.")
            return 0.0, 0.0, 0.0

        print(f"[INFO] Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            print("[WARN] Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            print(f"[WARN] Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            print(f"[INFO] Retry getting {description}...")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


# 테스트용 단독 실행 코드 (선택 사항)
if __name__ == '__main__':
    rclpy.init()
    detector = ObjectDetector()
    coords = detector.get_coords('cup')  # 예시
    print(f"[RESULT] 3D 위치: {coords}")
    rclpy.shutdown()
