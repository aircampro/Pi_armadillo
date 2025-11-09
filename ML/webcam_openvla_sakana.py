#!/usr/bin/env python3
# -*- coding: utf-8
# raspi webcam example with VLM or QR Code detector
#
from googletrans import Translator
import rich_click as click
import time
import cv2 as cv
from pygrabber.dshow_graph import FilterGraph
import threading

# Install minimal dependencies (`torch`, `transformers`, `timm`, `tokenizers`, ...)
# > pip install -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt
#
import torch
from transformers import AutoModelForVision2Seq, AutoProcessor

# choose the VLM vision language model
# https://github.com/openvla/openvla
OPENVLA = 0
# https://huggingface.co/SakanaAI
SAKANA = 1
INSTRUCTION="lift"
SUP=0.1
CAM_U_WIDTH = 320
CAM_U_HEIGHT = 240
CAM_U_FPS = 10

class threading_capture:
    def __init__(self, cap, max_queue_size=1):
        self.video = cap
        self.stopped = False
        self.frame = None
        self.active = True

    def start(self):
        thread=threading.Thread(target=self.update, daemon=True)
        thread.start()
        return self

    def update(self): 
        while self.active == True:
            try:
                if self.stopped:
                    return
          
                ok, frame = self.video.read()
                self.frame=frame
                #queue_from_cam.put(frame)

                if not ok:
                    self.stop()
                    return
            except cv.error:
                print("cv2.error")
                
            except KeyboardInterrupt:
                break     
        self.video.release()

    def read(self):
        if self.frame is None:
            return False, None
        else:
            return True, self.frame 

    def stop(self):
        self.stopped = True

    def release(self):
        self.stopped = True
        self.active = False
        #self.video.release()

    def isOpened(self):
        return self.video.isOpened()

    def get(self, i):
        return self.video.get(i)

class OpenVLMInterface:
    """This class encapsulates the OpenVLA/SakuraAI Agent's capabilities for remote action prediction."""

    def __init__(self, model_no=OPENVLA):
        if model_no == OPENVLA:
            self.model_name = "openvla/openvla-7b"
            self.processor = AutoProcessor.from_pretrained(self.model_name, trust_remote_code=True)
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model = AutoModelForVision2Seq.from_pretrained(
                self.model_name,
                attn_implementation="flash_attention_2",                                # [Optional] Requires `flash_attn`
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                trust_remote_code=True,
            ).to(self.device)
        if model_no == SAKANA:
            self.model_name = "SakanaAI/EvoVLM-JP-v1-7B"                                # this model predicts in japanese
            self.processor = AutoProcessor.from_pretrained(self.model_name)
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model = AutoModelForVision2Seq.from_pretrained(self.model_name)
                torch_dtype=torch.bfloat16,
            ).to(self.device)

    def predict_sakura(image, text):
        text = "<image>\n" + text
        messages = [
            {"role": "assistant", "content": "You are driving a car. Please answer the questions based on the image provided"},
            {"role": "user", "content": text},
        ]

        inputs = self.processor(text=text, images=image, return_tensors="pt").to(self.device)

        output_ids = self.model.generate(**inputs)
        output_ids = output_ids[:, inputs.input_ids.shape[1]:]
        generated_text = self.processor.batch_decode(output_ids, skip_special_tokens=True)[0].strip()

        return generated_text

    def predict_openvla(self, image, prompt, unnorm_key="bridge_orig"):
        """Predicts the robot's action based on the provided image and instruction."""
        inputs = self.processor(prompt, image).to(self.device, dtype=torch.bfloat16)
        return self.model.predict_action(**inputs, unnorm_key=unnorm_key, do_sample=False)

def read_cam(ins=None, qrcode=False, fmt="yuyv"):
    global INSTRUCTION
    if not ins == None:
        INSTRUCTION = ins
    # you can list all your cameras and choose from that list (i'm selecting the first one in our list below)
    camera_name = ["HD Pro Webcam C920", 'FHD Camera']
    camera_opencv_index = FilterGraph().get_input_devices().index(camera_name[0])
    camera = cv.VideoCapture(camera_opencv_index)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,CAM_U_WIDTH)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,CAM_U_HEIGHT)
    camera.set(cv.CAP_PROP_FPS,CAM_U_FPS)
    if not fmt.find("yuyv") == -1:
        camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('Y', 'U', 'Y', 'V'));
    elif not fmt.find("h264") == -1:
        camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '4'));
    camera.set(cv.CAP_PROP_BUFFERSIZE,1)
    capture = threading_capture(camera)
    capture.start()
    vla = OpenVLMInterface(OPENVLA)                                              # openvla model
    sak = OpenVLMInterface(SAKANA)                                               # sakana ai model ask questions about images to get detailed answers https://huggingface.co/SakanaAI
    translator = Translator()                                                    # translates jp answer to en
    time.sleep(SUP)                                                              # allow the camera to warmup
    try:
        while True:                                                              # capture frames from the camera
            ret, frame = capture.read()
            if ret == False:
                break
            image = frame.copy()
            output_image = image.copy()
            if qrcode == True:                                                   # do QR code detection
                detector = cv.QRCodeDetector()
                data, points, straight_qrcode = detector.detectAndDecode(image)
                if data:
                    print(f'decoded data: {data}')
                    for i in range(4):
                        cv.line(output_image, tuple(points[i][0]), tuple(points[(i + 1) % len(points)][0]), (0, 0, 255), 4)
                    cv.imshow('QR',output_image)
                    print(f'QR code version: {((straight_qrcode.shape[0] - 21) / 4) + 1}')
            else:                                                               # check image with vision language model
                prompt = f"In: What action should the robot take to {<INSTRUCTION>}?\nOut:" 
                sak_ret = sak.predict_sakura(image, prompt) 
                print(sak_ret)
                en_words = []
                for src in jp_words:
                    dst = translator.translate(src, src='ja', dest='en')
                    en_words.append(dst.text)
                print(en_words)
                vla_ret = vla.predict_openvla(image, prompt)
                print(vla_ret)            
            cv.imshow("Frame", image)                                           # show the frame that was captured and analysed
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):                                                 # if the `q` key was pressed, break from the loop
                break
    except KeyboardInterrupt:
        print("interrupted...")

    capture.release()

@click.command()
@click.option("--instruction", default='pick', help="instruction to advise on photo")
def main(instruction: str):
    read_cam(instruction)

if __name__ == '__main__':
    main()
