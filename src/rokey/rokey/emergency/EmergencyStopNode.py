import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pyaudio
import numpy as np

from dsr_msgs2.srv import DrlStop  # DSR 드라이버에서 사용하는 서비스
from rokey.emergency.WakeupWordEMStop import WakeupWord  # 너가 만든 클래스
from rokey.emergency.emergency_mike import  MicConfig, MicController

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.get_logger().info("🚨 EmergencyStopNode initialized.")

        # 오디오 설정
        self.buffer_size = 24000
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            input=True,
            frames_per_buffer=self.buffer_size,
        )
        self.mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1, record_seconds=5,
            fmt=pyaudio.paInt16, device_index=10, buffer_size=24000
        )
        self.mic = MicController(config=self.mic_config)
        self.mic.open_stream()
        # Wakeword 인식기 구성
        self.wakeup = WakeupWord(buffer_size=self.mic_config.buffer_size)
        self.wakeup.set_stream(self.stream)

        # DrlStop 서비스 클라이언트 설정
        self.client = self.create_client(DrlStop, 'drl/drl_stop')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting fsor drl/drl_stop service...')

        # 주기적 타이머로 wakeword 감지
        self.timer = self.create_timer(0.5, self.check_emergency)

    def check_emergency(self):
        if self.wakeup.is_wakeup():
            self.get_logger().warn("🚨 Emergency Wakeword Detected! Sending DrlStop")
            self.send_drl_stop()

    def send_drl_stop(self):
        req = DrlStop.Request()
        future = self.client.call_async(req)

        def callback(future):
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info("✅ DrlStop succeeded.")
                else:
                    self.get_logger().error("❌ DrlStop failed.")
            except Exception as e:
                self.get_logger().error(f"DrlStop service call failed: {e}")

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 EmergencyStopNode stopped by user.")
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.audio.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
