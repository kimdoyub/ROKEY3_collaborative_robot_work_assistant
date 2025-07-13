# run_weather_service.py
import rclpy
from rclpy.node import Node
from od_msg.srv import Srvchat
from dotenv import load_dotenv
from std_msgs.msg import String
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate, ChatPromptTemplate, SystemMessagePromptTemplate, HumanMessagePromptTemplate
import os
from ament_index_python.packages import get_package_share_directory
from gtts import gTTS

package_path = get_package_share_directory("rokey")
load_dotenv(dotenv_path=os.path.join(package_path, "resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

class RunWeatherService(Node):
    def __init__(self):
        super().__init__('run_weather_service')
        self.llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0.3, openai_api_key=openai_api_key)

        self.classify_prompt = PromptTemplate(
            input_variables=["user_input"],
            template="""
        일기예보를 보고 오늘 무슨날씨인지 보기중에서 하나 골라줘

        <주제 리스트>
        - 맑음, 흐림, 비, 눈

        <출력예시>
        <예시>
            - 입력: 6월 26일, 현재 시간: 오후 2시 30분 날씨입니다.  
            오늘 날씨는 대체로 맑겠습니다. 낮 최고 기온은 29°C, 최저 기온은 20°C로 예상됩니다.  
            바람은 남서풍이 초속 2~4m로 불겠으며, 미세먼지 농도는 보통 수준입니다.  
            습도는 약 60%로 야외 활동에 큰 무리는 없겠습니다.  
            덥지만 바람이 불어 통기성 좋은 얇은 옷차림을 추천드립니다.
            출력: 맑음

        <출력 형식>aWQQ
        - 오직 하나의 주제를 선택해 주제리스트에 있는 단어를 출력하세요(예: 맑음)
        <사용자 입력>
        "{user_input}"  
        """
        )

        self.srv = self.create_service(Srvchat, 'run_weather', self.handle_run_weather)
        self.get_logger().info('🌤️ [run_weather] 서비스 대기 중...')
        
        self.weather_summary_pub = self.create_publisher(String, '/weather_word', 10)

    def weather_summary(self, user_input):
        chain = LLMChain(llm=self.llm, prompt=self.classify_prompt)
        result = chain.invoke({"user_input": user_input})
        self.get_logger().info(f"📢 퍼블리시된 요약 날씨: {result}")
        return result["text"].strip().lower()

    def handle_run_weather(self, request, response):
        self.get_logger().info(f"✅ [weather] 요청 수신: {request.result}")
        text = request.result
        tts = gTTS(text=text, lang='ko')
        tts.save("output.mp3")
        os.system("mpg123 output.mp3")
        weather_topic = self.weather_summary(request.result)
        msg = String()
        msg.data = weather_topic
        self.get_logger().info(f"📢 퍼블리시된 요약 날씨: {weather_topic}")
        # 퍼블리시
        self.weather_summary_pub.publish(msg)
        response.success = True
        response.feedback = "날씨 응답 완료"
        return response


def main():
    rclpy.init()
    node = RunWeatherService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


