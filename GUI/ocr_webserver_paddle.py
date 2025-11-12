#!/usr/bin/env python3
# paddle ocr with gradio server
#
import base64
from io import BytesIO
import gradio as gr
import torch
from PIL import Image
from modelscope import AutoModelForCausalLM, AutoProcessor
import sys

if len(sys.argv[0]) >= 1:
    SERVER_IP=str(argv[1])
else:
    SERVER_IP="0.0.0.0"
if len(sys.argv[0]) >= 2: 
    SERVER_PORT=int(argv[2])
else:    
    SERVER_PORT=3389

"""
Gradio server for paddle OCR
"""

class PaddleOCRInterface:
    """This class encapsulates the https://modelscope.cn/models/PaddlePaddle/PaddleOCR-VL OCR for an image."""

    def __init__(self, model_name="PaddlePaddle/PaddleOCR-VL"):
        self.processor = AutoProcessor.from_pretrained(model_name, trust_remote_code=True)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = AutoModelForCausalLM.from_pretrained(model_name, trust_remote_code=True, torch_dtype=torch.bfloat16).to(self.device).eval()
        self.CHOSEN_TASK = "ocr"                                                                               # Options: 'ocr' | 'table' | 'chart' | 'formula'
        self.PROMPTS = {
            "ocr": "OCR:",
            "table": "Table Recognition:",
            "formula": "Formula Recognition:",
            "chart": "Chart Recognition:",
        }

    def predict_action(self, image_base64, instruction, unnorm_key=None, image_path=None):
        """ perform ocr on the image using paddle."""
        if image_base64:
            image = Image.open(BytesIO(base64.b64decode(image_base64)))                                         # Assume it's a base64 image
        elif image_path:
            image = Image.open(image_path)                                                                      # Assume it's an uploaded image
        else:
            raise ValueError("Either an uploaded image or a base64 image must be provided.")

        messages = [
            {"role": "user",         
             "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": self.PROMPTS[self.CHOSEN_TASK]},
                ]
            }
        ]
        inputs = self.processor.apply_chat_template(
            messages, 
            tokenize=True, 
            add_generation_prompt=True, 	
            return_dict=True,
            return_tensors="pt"
        ).to(self.device)
        outputs = self.model.generate(**inputs, max_new_tokens=1024)
        outputs = self.processor.batch_decode(outputs, skip_special_tokens=True)[0]
        print(outputs)
        return outputs

def create_interface():
    """Creates and returns a Gradio Interface for the OpenVLA robot action prediction."""
    paddocr = PaddleOCRInterface()
    return gr.Interface(
        fn=paddocr.predict_action,
        inputs=[
            gr.Textbox(label="Base64 Image (using API) or upload image below.", visible=False),
            gr.Image(label="Upload Image", type="filepath"),
        ],
        outputs=gr.Textbox(label="OCR Result"),
        title="Paddle OCR",
        description="either upload an image or provide a base64-encoded image with API.",
    )

# Launch the server on port 3389
if __name__ == "__main__":
    interface = create_interface()
    interface.launch(server_name=SERVER_IP, server_port=SERVER_PORT)
