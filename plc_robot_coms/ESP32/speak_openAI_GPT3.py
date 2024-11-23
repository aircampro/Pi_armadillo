import alexa_like_whisper
import aques_talk
import openai
import socket
import os
import sys

def openAIQuery(query):
    response = openai.Completion.create(
    model="text-davinci-003",
    prompt=query,
    temperature=0.9,
    max_tokens=150,
    top_p=1,
    frequency_penalty=0.0,
    presence_penalty=0.6,
    )

    return response['choices'][0]['text']

def ChatGPTQuery(query):
    response = openai.ChatCompletion.create(
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": "あなたは役に立つアシスタントです。"},
        {"role": "user", "content": query}
    ]
    )

    return response["choices"][0]["message"]["content"]

class Waiting():
    def run(self, result):
        return result

class Talking():
    def __init__(self, func):
        self.aqueswriter = aques_talk.AquesTalkWriter()
        self.func = func

    def run(self, result):
        try:
            self.aqueswriter.response = self.func(result)
        except Exception as e:
            print(f'Exception : {str(e)}')

        message = self.aqueswriter.write()

        return message

class M5State():
    def __init__(self, func):
        self.waiting = Waiting()
        self.talking = Talking(func)
        self.state = self.waiting

    def change_state(self, result):
        if result == "Sleep":
            self.state = self.waiting
        elif result != "Sleep" and result != "On recording..." and result != "Wake":
            self.state = self.talking

    def run(self, result):
        return self.state.run(result)

class Sender():
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, data, ipaddress, port):
        self.s.sendto(bytes(data, 'utf-8'), (ipaddress,port))

    def close(self):
        self.s.close()

if __name__ == "__main__":
    # Modelsizes on whisper
    MODELSIZES = ['tiny', 'base', 'small', 'medium', 'large']

    # AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)
    ACCESS_KEY = os.environ["PICOVOICE_API_KEY"]
    KEYWORD_PATH = ['']

    # Recording Time(s)
    RECORDING_TIME = 5

    alexa_like = alexa_like_whisper.AlexaLikeWhisper(ACCESS_KEY, KEYWORD_PATH, MODELSIZES[3], RECORDING_TIME)

    # Parameters on communication about stackchan read them from enviroment variables as defined
    M5_IPADDRESS = os.environ["M5STACK_IP_ADDRESS"]
    M5_PORTNUMBER = 50100
    sender = Sender()

    openai.api_key = os.environ["OPENAI_API_KEY"]

    if (len(sys.argv) >= 1):  
        # use GPT-3
        m5state = M5State(openAIQuery) 
    else:        
        # use ChatGPT
        m5state = M5State(ChatGPTQuery)

    try:
        while True:
            result = alexa_like.run()
            m5state.change_state(result)
            result = m5state.run(result)

            sender.publish(result, M5_IPADDRESS, M5_PORTNUMBER)

    except KeyboardInterrupt:
        sender.close()