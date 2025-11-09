#!/usr/bin/env python3
# -*- coding: utf-8
# raspi cam example with VLM or QR Code detector
#
from googletrans import Translator
import rich_click as click
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv

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

def read_cam(ins=None, qrcode=False):
    global INSTRUCTION
    if not ins == None:
        INSTRUCTION = ins
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    vla = OpenVLMInterface(OPENVLA)                                          # openvla model
    sak = OpenVLMInterface(SAKANA)                                           # sakana ai model ask questions about images to get detailed answers https://huggingface.co/SakanaAI
    translator = Translator()                                                # translates jp answer to en
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
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
        # show the frame that was captured and analysed
        cv.imshow("Frame", image)
        key = cv.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

@click.command()
@click.option("--instruction", default='pick', help="instruction to advise on photo")
def main(instruction: str):
    read_cam(instruction)

if __name__ == '__main__':
    main()

