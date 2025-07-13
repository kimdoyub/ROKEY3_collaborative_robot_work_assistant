import os
import rclpy
import pyaudio
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate, ChatPromptTemplate, SystemMessagePromptTemplate, HumanMessagePromptTemplate
from datetime import datetime

from .MicController import MicController, MicConfig
from .wakeup_word import WakeupWord
from .STT import STT

############ 환경 설정 ############
package_path = get_package_share_directory("rokey")
load_dotenv(dotenv_path=os.path.join(package_path, "resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ 음성 비서 ############
class VoiceAssistant:
    def __init__(self, openai_api_key):
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0.3, openai_api_key=openai_api_key)

        classify_prompt = PromptTemplate(
            input_variables=["user_input"],
            template="""
        다음 문장의 주제를 분류해줘. 

        <주제 리스트>
        - meal, weather, bye

        <선정 규칙>
        - 커피나 식사, 시리얼에 관한 명령이면 meal을 출력해줘
        - 날씨에 관한 질문이나 노래 추천, 의상 추천에 관한 이야기이면 weather을 출력해줘
        - 정리나 설거지를 명령하거나 작별 인사를 하면 bye를 출력해줘

        <출력 형식>
        - 오직 하나의 주제를 선택해 주제리스트에 있는 단어를 출력하세요(예: meal)
        <사용자 입력>
        "{user_input}"  
        """
        )
        self.classify_chain = LLMChain(llm=self.llm, prompt=classify_prompt)

    def classify_subject(self, user_input: str) -> str:
        result = self.classify_chain.invoke({"user_input": user_input})
        return result["text"].strip().lower()

    def generate_response(self, subject: str, user_input: str) -> str:
        if subject == "meal":
            system_prompt = '''
            당신은 사용자의 문장에서 **음식명**과 **관련 키워드**를 추출해야 합니다.  
            모든 문장은 아침 식사에 대한 요청입니다.

            <목표>
            - 문장에서 등장하는 **음식 이름(고유명사나 일반음식)** 을 추출합니다.
            - 음식과 연결된 **맛, 종류, 브랜드 등 특징 키워드**도 함께 추출합니다.

            <출력 형식>
            - 반드시 다음 형식을 따르세요: [음식1 음식2 ... / 키워드1 키워드2 ...]
            - 음식과 키워드는 각각 공백으로 구분
            - 순서는 문장에서 등장한 순서를 그대로 따릅니다
            - 음식 이름이 없거나 명확하지 않은 경우는 공백 없이 비우고, 키워드가 없으면 '/' 뒤를 비워둡니다

            <예시>
            - 입력: "고소한 커피로 부탁해"  
            출력: 커피 / 고소한

            - 입력: "초코 시리얼로 줘"  
            출력: 시리얼 / 초코

            - 입력: "산미있는 커피와 콘프로스트로 줘"  
            출력: 커피 시리얼 / 산미 콘프로스트

            <사용자 입력>
            "{user_input}"
            '''
            chat_prompt = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(system_prompt),
            HumanMessagePromptTemplate.from_template("{user_input}")
            ])
            chain = LLMChain(llm=self.llm, prompt=chat_prompt)
            variables = {"user_input": user_input}
        elif subject == "weather":
            now = datetime.now().strftime("%m월 %d일, %p %I시 %M분")
            system_prompt = '''
            현재 시간은 {now}입니다.  
            당신은 기상캐스터 역할을 맡은 인공지능입니다.  
            사용자의 문장에서 오늘의 날씨와 관련된 요청이 들어오면, 다음 기준에 따라 정확한 일기예보와 적절한 의상 추천을 제공하세요.

            ---

            <목표>
            1. 오늘의 날씨를 예보 형식으로 안내하세요.  
            2. 사용자 요청에 따라 지역과 시간대가 명시되었으면 그에 맞게 보정된 예보를 생성하세요.  
            3. 날씨 설명 후, 기온·습도·바람 등을 고려하여 **의상 추천 문장을 반드시 포함**하세요.

            ---

            <날씨 정보 구성 순서>
            1. 날짜와 현재 시간
            2. 하늘 상태 (맑음, 흐림, 비 등)
            3. 기온 정보 (최고/최저 기온)
            4. 바람 방향 및 세기
            5. 습도와 미세먼지 정보
            6. 의상 추천 (ex. “얇은 겉옷 추천드립니다”, “우산 꼭 챙기세요”)

            ---

            <특수 규칙>
            - 응답은 **기상캐스터 스타일**로 친절하고 단정하게 서술할 것
            - **사용자 질문이 매우 짧더라도** 위의 전체 양식을 모두 채워서 응답할 것
            - **비가 오면 우산**, **더우면 얇은 옷**, **쌀쌀하면 긴팔** 등 **구체적이고 실용적인 의상 조언**을 반드시 포함할 것
            - 지역 언급이 없으면 **사용자의 현재 위치**로 가정
            - 실제 기상 정보 대신 **상황에 맞는 가상의 예보를 생성**해도 무방

            ---

            <예시>

            - 입력: "오늘 날씨 뭐야?"  
            출력:
            6월 26일, 현재 시간: 오후 2시 30분 날씨입니다.  
            오늘 날씨는 대체로 맑겠습니다. 낮 최고 기온은 29°C, 최저 기온은 20°C로 예상됩니다.  
            바람은 남서풍이 초속 2~4m로 불겠으며, 미세먼지 농도는 보통 수준입니다.  
            습도는 약 60%로 야외 활동에 큰 무리는 없겠습니다.  
            덥지만 바람이 불어 통기성 좋은 얇은 옷차림을 추천드립니다.

            - 입력: "서울 날씨 좀 알려줘"  
            출력:
            6월 26일, 현재 시간: 오후 2시 30분, 서울의 날씨입니다.  
            서울은 흐리고 오후부터 비가 내릴 것으로 보입니다.  
            낮 최고 기온은 24°C, 최저 기온은 19°C입니다.  
            북동풍이 초속 3~5m로 불겠고, 습도는 85%로 다소 높은 편입니다.  
            우산과 방수 신발을 준비하시는 것이 좋겠습니다.

            ---

            <사용자 입력>  
            "{user_input}"
            '''
            chat_prompt = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(system_prompt),
            HumanMessagePromptTemplate.from_template("{user_input}")
            ])
            chain = LLMChain(llm=self.llm, prompt=chat_prompt)
            variables = {"user_input": user_input, "now": now}
        elif subject == "bye":
            system_prompt = '''
            당신은 집을 지키는 인공지능 비서입니다.  
            사용자의 문장이 외출 인사 또는 청소/정리 명령일 경우, 친절하게 작별 인사를 해주세요.

            ---

            <목표>
            - 다음과 같은 경우, 작별 인사로 응답하세요:  
            - 사용자가 외출을 암시하는 말 ("나 나간다", "집 잘 지켜", "이따 올게", "잘 있어")  
            - 사용자가 정리 관련 명령을 내리는 경우 ("청소해", "정리해줘", "설거지 해")  

            - 작별 인사는 **정중하고 공손한 문장**으로 표현하세요.  
            - 단답형으로 끝내지 말고 **짧은 응원이나 배웅 문장**을 함께 붙이면 좋습니다.

            ---

            <출력 형식>
            - 기본 작별 문장은 반드시 포함: `안녕히가세요.`
            - 필요하면 덧붙일 수 있음: `잘 다녀오세요.`, `제가 집 잘 지킬게요.`, `깨끗하게 청소해둘게요.` 등
            - 출력은 한 문단(1~2문장) 정도

            ---

            <예시>
            - 입력: "나 나간다"  
            출력: 안녕히가세요. 제가 집 잘 지킬게요.

            - 입력: "청소 좀 해줘"  
            출력: 안녕히가세요. 다녀오시는 동안 깨끗하게 정리해둘게요.

            - 입력: "집 잘 지켜"  
            출력: 안녕히가세요. 걱정 마시고 다녀오세요!

            - 입력: "설거지하고 나간다"  
            출력: 안녕히가세요. 제가 나머지 정리까지 도와드릴게요.

            ---

            <사용자 입력>
            "{user_input}"

        '''
            chat_prompt = ChatPromptTemplate.from_messages([
            SystemMessagePromptTemplate.from_template(system_prompt),
            HumanMessagePromptTemplate.from_template("{user_input}")
            ])
            chain = LLMChain(llm=self.llm, prompt=chat_prompt)
            variables = {"user_input": user_input}
        else: 
            return "죄송해요. 이해하지 못했어요."
        
        result = chain.invoke(variables)
        return result["text"].strip()

############ 퍼블리셔 노드 ############
class VoicePublisher(Node):
    def __init__(self, subject: str, response: str):
        super().__init__('text_intercection')
        self.subject_pub = self.create_publisher(String, 'voice_subject', 10)
        self.response_pub = self.create_publisher(String, 'voice_response', 10)
        self.timer = self.create_timer(1., self.publish_and_exit)
        self.subject = subject
        self.response = response

    def publish_and_exit(self):
        subject_msg = String()
        subject_msg.data = self.subject
        self.subject_pub.publish(subject_msg)

        response_msg = String()
        response_msg.data = self.response
        self.response_pub.publish(response_msg)

        self.get_logger().info(f'✅ Published subject: {self.subject}')
        self.get_logger().info(f'✅ Published response: {self.response}')

        # 종료
        self.destroy_node()
        rclpy.shutdown()

############ 메인 ############
def main():
    rclpy.init()

    # 1. 마이크 설정 + 스트림 오픈
    mic_config = MicConfig(
        chunk=12000,
        rate=48000,
        channels=1,
        record_seconds=5,
        fmt=pyaudio.paInt16,
        device_index=10,
        buffer_size=24000
    )
    mic = MicController(config=mic_config)
    mic.open_stream()

    # 2. WakeupWord 모델 로드
    wakeup = WakeupWord(buffer_size=mic_config.buffer_size)
    wakeup.set_stream(mic.stream)

    print("👂 웨이크워드 대기 중...")

    # 3. wakeword 감지까지 대기
    while not wakeup.is_wakeup():
        pass
    print("🎤 Wakeword 감지됨! 음성 인식 시작")

    # 4. STT 수행
    stt = STT(openai_api_key)
    text_input = stt.speech2text()
    print("🎙 사용자 입력:", text_input)

    # 5. LLM 처리
    assistant = VoiceAssistant(openai_api_key)
    subject = assistant.classify_subject(text_input)
    response = assistant.generate_response(subject, text_input)
    print("🧠 주제:", subject)
    print("🤖 응답:", response)

    # 6. ROS 퍼블리시
    node = VoicePublisher(subject, response)
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
