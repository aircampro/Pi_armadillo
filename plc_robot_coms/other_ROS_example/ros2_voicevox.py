#!/usr/bin/env python3
#
# Example of talking robot on ROS2 using the voicevox package for ROS2 humble
# ref :- https://github.com/GAI-313/voicevox_ros2/blob/humble/voicevox_ros2/voicevox_ros2/voicevox_ros2_tts.py use VoiceVox with ROS2
# VOICEVOX_ROS2 is a text-to-speech package that works with ROS2 humble
#
# to install do the follwing :-
# mkdir colcon_ws/src
# cd colcon_ws/src
# git clone https://github.com/GAI-313/voicevox_ros2.git
# cd colcon_ws
# colcon build --packages-up-to voicevox_ros2
# source ~/.bashrc
# source colcon_ws/install/setup.bash
# to start the core
# ros2 run voicevox_ros2 voicevox_ros2_core
#
# When this node starts, the following topics are published:
# /voicevox_ros2/speaker
# /voicevox_ros2/status
#
# to test start a publisher
# ros2 run voicevox_ros2 pub
# ros2 topic pub --once /voicevox_ros2/speaker voicevox_ros2_interface/msg/Speaker "{text: 'this is a test from publisher', id: 1}"
#
# via a service
# ros2 run voicevox_ros2 srv
# ros2 service call /voicevox_ros2/speaker_srv voicevox_ros2_interface/srv/Speaker "{text: 'this service speaks', id: 36}"
#
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.srv import Speaker
from enum import Enum

# the following classes describe the possible voices you can use 
class Shikoku(Enum):
    normal = 2
    Amama = 0
    Tsuntsun = 6
    sexy = 4	
    Whisper = 36
    Hisohiso = 37	
	
class Zundamon(Enum):
    normal = 3
    Amama = 1
    Tsuntsun = 7
    sexy = 5	
    Whisper = 22
    Hisohiso = 38
    HeroheroHélo = 75
    Namidam	= 76

class KasukabeTsumugi(Enum):
    normal = 8
    RainyWeather = 10

class RitsuHamine(Enum):
    normal = 9
    queen = 65

class TakehiroGenno(Enum):
    normal = 11
    joy = 39
    Tungire	= 40
    grief = 41	

class TorataroShirakami(Enum):
    Usually = 12
    Yay = 32
    Jumpy = 33
    Oko = 34	
    Bien = 35

class RyuseiAoyama(Enum):
    normal = 13
    zeal = 81
    pout = 82
    joy = 83	
    moist = 84
    grief = 85
    whisper = 86

class Himari(Enum):
    normal = 14
	
class KyushuSora(Enum):
    normal = 16
    Amama = 15
    Tsuntsun = 18
    sexy = 17	
    Whisper = 19

class MsMochiko(Enum):
    normal = 20
    SexyAnko = 66
    weeping = 77
    anger = 78
    joy = 79
    carefree = 80

class Kenzaki(Enum):
    normal = 21
	
class WhiteCUL(Enum):
    normal = 23
    Fun = 24
    Sad = 25
    Bien = 26
    GooniHuman = 27
    Plush = 28

class No7(Enum):
    normal = 29
    announcement = 30
    Storytelling = 31

class ChibiShikiGrandfather(Enum):
    normal = 32

class MikoSakurakafather(Enum):
    normal = 43	
    Secondform = 44
    Lolita = 45

class Sayo(Enum):
    normal = 46

class NurseRoboTypeT(Enum):
    normal = 47	
    Effortlessly = 48
    fear = 49
    SecretStory = 50	

# †Holy Knight Red Cherry Blossom†
class HolyKnightRedCherryBlossom(Enum):
    normal = 51

class ShujiYakumatsu(Enum):
    normal = 52

class KirgashimaSorin(Enum):
    normal = 53

class HarukaNana(Enum):
    normal = 54

class CatMessenger(Enum):
    normal = 55	
    Ochitsuki = 56
    cheerfully = 57

class CatMessengerBee(Enum):
    normal = 58	
    Ochitsuki = 59
    shyness = 60
	
class ChineseRabbit(Enum):
    normal = 61
    astonishment = 62
    Fear = 63
    Herohero = 64

class MaronKurita(Enum):
    normal = 67

class Aierutan(Enum):
    normal = 68	

class ManbetsuHanamaru(Enum):
    normal = 69
    healthy = 70
    Whisper = 71
    Buriko = 72
    Boy = 73
	
class KotoWingNia(Enum):
    normal = 74
	
def say(node, text, id=3, wait=True):
    def send_req(text, id):
        req.text = text
        req.id = id
        future = cli.call_async(req)

        if wait:
            rclpy.spin_until_future_complete(node, future)
            return future.result()

    try:
        cli = node.create_client(Speaker, "voicevox_ros2/speaker_srv")
        while not cli.wait_for_service(timeout_sec=10.0):
            node.get_logger().warn("voicevox_ros2 is not available. wait again")
        req = Speaker.Request()

        res = send_req(text, id)
    except KeyboardInterrupt:
        pass
    except:
        import traceback
        node.get_logge().error(traceback.print_exc())

def _sample_executor():
    rclpy.init()
    node = Node('voicevox_ros2_sample')

    node.get_logger().warn("This is sample node.")
    say(node, text='test sample execution')

# debug
if __name__ == '__main__':
    rclpy.init()
    node = Node('voicevox_ros2_sample')

    # should speak japanese
    say(node, text='こんにちは！ボイスボックスロスへようこそ')
    say(node, text='キャラクターIDを変更すると、しゃべるキャラクターが変わります', id=Shikoku.Hisohiso.value)
    say(node, text='新たなキャラクターID、すんだもん、ヘロヘロと、', id=Zundamon.Namidam.value)
    say(node, text='すんだもん、なみだめ、', id=RitsuHamine.queen.value)
    say(node, text='中国うさぎ、ノーマル', id=ChineseRabbit.Herohero.value)
    say(node, text='中国うさぎ、おどろき', id=ManbetsuHanamaru.Boy.value)
    say(node, text='中国うさぎ、こわがり', id=HolyKnightRedCherryBlossom.normal.value, True)
    say(node, text='中国うさぎ、へろへろ', id=MsMochiko.SexyAnko.value)
    say(node, text='キャラクターIDを変更すると、しゃべるキャラクターが変わります', id=WhiteCUL.GooniHuman.value)
	
    # and also english
    say(node, text='i speaks english', id=TorataroShirakami.Bien.value)
    say(node, text='i is speaking english', id=KyushuSora.sexy.value)
    say(node, text='i speak english', id=MikoSakurakafather.Lolita.value)
    say(node, text='i am speaking in english', id=NurseRoboTypeT.fear.value)
    say(node, text='i am a boy from england', id=ManbetsuHanamaru.Boy.value, True)	
    say(node, text='i am a girl from england', id=CatMessengerBee.shyness.value, True)	
