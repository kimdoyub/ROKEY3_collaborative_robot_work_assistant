import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import random
from playsound import playsound

class WeatherAudioPlayer(Node):
    def __init__(self):
        super().__init__('weather_audio_player')
        self.subscription = self.create_subscription(
            String,
            '/weather_word',
            self.listener_callback,
            10
        )
        self.get_logger().info("🎧 WeatherAudioPlayer 노드 실행 중... '/weather_word' 구독 대기")

    def listener_callback(self, msg):
        weather = msg.data.strip()
        weather_path = os.path.join('/home/rokey/ros2_ws/play_list', weather)
        filename = os.path.join(weather_path,random.choice(os.listdir(weather_path)))

        self.get_logger().info(f"🔔 수신한 요약 날씨: {weather}")
        
        if os.path.exists(filename):
            self.get_logger().info(f"🔊 {filename} 재생 중...")
            playsound(filename)
        else:
            self.get_logger().warn(f"❌ 오디오 파일 '{filename}'이 존재하지 않습니다.")


def main():
    rclpy.init()
    node = WeatherAudioPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
