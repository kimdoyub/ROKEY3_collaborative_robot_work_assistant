import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

import cv2
from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from pick_and_place_text.realsense import ImgNode
from pick_and_place_text.yolo import YoloModel


PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )

        self.timer = self.create_timer(0.1, self.visualize_detection)  # 10Hz 시각화 타이머
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        rclpy.spin_once(self.img_node)
        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0

        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )

    def visualize_detection(self):
        """YOLO 객체 감지를 수행하고 bounding box를 시각화"""
        rclpy.spin_once(self.img_node)
        frame = self.img_node.get_color_frame()
        if frame is None:
            return

        results = self.model.model(frame, verbose=False)
        annotated_frame = results[0].plot()  # YOLO가 bounding box 표시해줌
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
