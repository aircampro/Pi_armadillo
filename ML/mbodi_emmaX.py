#!/usr/bin/env python3
# VLM example using emma-X
# ref:- https://github.com/mbodiai/embodied-agents/blob/main/examples/2_openvla_motor_agent_example.py
# example is using emma-X agent instead of openVLA
#
# Install minimal dependencies (`torch`, `transformers`, `timm`, `tokenizers`, ...)
# > pip install -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt
#
from modelscope import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch
from mbodied.agents.sense.audio.audio_agent import AudioAgent
from mbodied.robots import SimRobot

def main():
    audio = AudioAgent(use_pyaudio=False)  
    robot = SimRobot()
    device = "cuda:0" if torch.cuda.is_available() else "cpu"

    # Recieve inputs.
    task_label = audio.listen()                                                                           # listen for the voice request
    image: Image.Image = robot.capture()                                                                  # capture the vision image input

    # Load Emma-X
    vla = AutoModelForVision2Seq.from_pretrained(
        "declare-lab/Emma-X",
        attn_implementation="flash_attention_2",                                                          # [Optional] Requires `flash_attn`
        torch_dtype=torch.bfloat16, 
        low_cpu_mem_usage=True, 
        trust_remote_code=True
    ).to(device)
    processor = AutoProcessor.from_pretrained("declare-lab/Emma-X", trust_remote_code=True)

    prompt, image = processor.get_prompt(task_label, image)
    inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
    # Predict Action (action is a 7 dimensional vector to control the robot)
    action, reasoning = vla.generate_actions(inputs, processor.tokenizer, do_sample=False, max_new_tokens=512)
    print("action", action)
    print("reasoning", reasoning)

    audio.speak(f"doing {task_label}")
    robot.do(action)

if __name__ == '__main__':
    main()