import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rokey.meal.realsense import ImgNode
from rokey.meal.yolo1 import YoloModel

PACKAGE_NAME = 'rokey'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetector:
    def __init__(self):
        self.img_node = ImgNode()
        self.model = self._load_model('yolo')
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        print("ObjectDetector initialized.")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def get_coords(self, target):
        """
        외부에서 호출할 메인 함수.
        대상 객체의 3D 좌표를 반환합니다.
        """
        print(f"Received target: {target}")
        coords = self._compute_position(target)
        answer = [float(x) for x in coords]
        return answer

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        # 최신 이미지 데이터 갱신
        rclpy.spin_once(self.img_node)

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            print("No detection found.")
            return 0.0, 0.0, 0.0

        print(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            print("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(
            self.img_node.get_depth_frame, "depth frame"
        )
        try:
            return frame[y, x]
        except IndexError:
            print(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """
        getter 함수가 유효한 데이터를 반환할 때까지 spin하며 재시도합니다.
        """
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            print(f"Retry getting {description}.")
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

