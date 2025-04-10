# This example uses a rock5 cpu with the follwoing AI acceleration module on PCIe
# Hailo	Hailo-8 M.2 AI Acceleration Module
# based on infor from https://qiita.com/Kyaramel/items/659bc3dafc139c0326e1
#
#sudo apt update
#sudo apt upgrade -y
#sudo apt install -y cmake dkms
#
#git clone -b v4.19.0 https://github.com/hailo-ai/hailort-drivers.git
#cd hailort-drivers/linux/pcie
#make all
#sudo make insttall_dkms
#
#cd ~/hailort-drivers
#./download_firmware.sh
#sudo mv hailo8_fw.4.19.0.bin /lib/firmware/hailo/hailo8_fw.bin
#sudo cp linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
#
#sudo cp linux/pcie/hailo_pci.conf /etc/modprobe.d/
#sudo vim /etc/modprobe.d/hailo_pci.conf
#
#hailo_pci.conf
# Modprobe information used for Hailo PCIe driver, edit this file when needed.
#
# Disables automatic D0->D3 transition, should be set on platforms that
# does not support such transition.
#options hailo_pci no_power_mode=N # この行のコメントを外す
#
# Determines the maximum DMA descriptor page size (must be a power of 2), needs to be
# set on platforms that does not support large pcie transactions.
#options hailo_pci force_desc_page_size=4096 # この行のコメントも外す
#
#$ sudo apt install ./hailort_4.19.0_arm64.deb
#$ hailortcli fw-control identify　# このコマンドでHailo-8が認識されていることを確
#
#pip install hailort-4.19.0-cp310-cp310-linux_aarch64.whl
# yolo model being used is here
#mkdir models
#cd models
#wget https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.13.0/hailo8/yolov10x.hef
#
#
import matplotlib.pyplot as plt
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
from hailo_platform import (HEF, ConfigureParams, FormatType, HailoSchedulingAlgorithm, HailoStreamInterface,
                           InferVStreams, InputVStreamParams, InputVStreams, OutputVStreamParams, OutputVStreams,
                           VDevice)

# if this option is set then look_for traffic lights
# look_for=LOOK_FOR_OBJECT
#
LOOK_FOR_OBJECT="traffic light"

class HailoYOLODetector:
    def __init__(self, hef_path, threshold=0.5, min_score=0.45, use_pil=False, look_for=None, fo=1):
        self.hef_path = hef_path
        self.threshold = threshold
        self.min_score = min_score
        self.use_pil = use_pil
        self.target = self._initialize_vdevice()
        self.hef = HEF(self.hef_path)
        self.input_vstream_info, self.output_vstream_info = self._get_vstream_info()
        self.network_group = self._configure_network()
        self.look_for = look_for                                                           # object to look for
        self.first_one = fo                                                                # if 1 only return first object
       
        self.class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
                            'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
                            'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
                            'scissors', 'teddy bear', 'hair drier', 'toothbrush']

    def _initialize_vdevice(self):
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.NONE
        return VDevice(params=params)

    def _get_vstream_info(self):
        input_vstream_info = self.hef.get_input_vstream_infos()[0]
        output_vstream_info = self.hef.get_output_vstream_infos()[0]
        print("input layer", input_vstream_info)
        print("output layer", output_vstream_info)
        print("input shape - ", input_vstream_info.shape)
        print("output shape - ", output_vstream_info.shape)
        return input_vstream_info, output_vstream_info

    def _configure_network(self):
        configure_params = ConfigureParams.create_from_hef(hef=self.hef, interface=HailoStreamInterface.PCIe)
        network_groups = self.target.configure(self.hef, configure_params)
        return network_groups[0]

    def _extract_detections(self, input_data):
        boxes = []
        scores = []
        classes = []
        num_detections = 0
        for i, detection in enumerate(input_data):
            if len(detection) == 0:
                continue
            for j in range(len(detection)):
                bbox = np.array(detection)[j][:4]
                score = np.array(detection)[j][4]
                if score < self.threshold:
                    continue
                else:
                    boxes.append(bbox)
                    scores.append(score)
                    classes.append(i)
                    num_detections = num_detections + 1
        return {'detection_boxes': [boxes],
                'detection_classes': [classes],
                'detection_scores': [scores],
                'num_detections': [num_detections]}

    def _post_nms_infer(self, raw_detections):
        detections = self._extract_detections(raw_detections[self.output_vstream_info.name][0])
        return detections

    def _draw_detection(self, image, d, c, s, scale_factor):
        label = self.class_names[c] + ": " + "{:.2f}".format(s) + '%'
        ymin, xmin, ymax, xmax = d
        # look for a traffic light and return its data
        if not self.look_for == None:
            obj_roi = []
            if not label.find(self.look_for) == -1:
                obj_roi = [ label, ymin, xmin, ymax, xmax ]    
  
        index = 0
        for i, element in enumerate(self.class_names):
            if element == self.class_names[c]:
                index = i
                break
        flag = i % 4
        color = (255, 0, 0)
        if flag == 0:
          color = (127, 0, 0)
        elif flag == 1:
          color = (0, 127, 0)
        elif flag == 2:
          color = (0, 0, 127)
        elif flag == 3:
          color = (127, 127, 127)

        if self.use_pil:
            # PIL drawing
            draw = ImageDraw.Draw(image)
            font = ImageFont.truetype("LiberationSans-Regular.ttf", size=18)
            draw.rectangle([(xmin * scale_factor, ymin * scale_factor), (xmax * scale_factor, ymax * scale_factor)], outline=color, width=4)
            text_bbox = draw.textbbox((xmin * scale_factor + 4, ymin * scale_factor + 4), label, font=font)
            text_bbox = list(text_bbox)
            text_bbox[0] -= 4
            text_bbox[1] -= 4
            text_bbox[2] += 4
            text_bbox[3] += 4
            draw.rectangle(text_bbox, fill=color)
            draw.text((xmin * scale_factor + 4, ymin * scale_factor + 4), label, fill="black", font=font)
        else:
            # OpenCV drawing
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            cv2.rectangle(image, (int(xmin * scale_factor), int(ymin * scale_factor)), (int(xmax * scale_factor), int(ymax * scale_factor)), color, thickness)
            text_size, _ = cv2.getTextSize(label, font, font_scale, thickness)
            text_w, text_h = text_size
            cv2.rectangle(image, (int(xmin * scale_factor), int(ymin * scale_factor)), (int(xmin * scale_factor) + text_w, int(ymin * scale_factor) - text_h), color, -1)
            cv2.putText(image, label, (int(xmin * scale_factor), int(ymin * scale_factor)), font, font_scale, (0, 0, 0), thickness)

        if self.look_for == None:
            return image
        else:
            return image, obj_roi

    # processes all if look_for is None otherwise until until look_for has been seen
    #
    def _post_process(self, detections, image, width, height, scale_factor=1):
        boxes = np.array(detections['detection_boxes'])[0]
        classes = np.array(detections['detection_classes'])[0].astype(int)
        scores = np.array(detections['detection_scores'])[0]

        obj_rois = []
        for idx in range(np.array(detections['num_detections'])[0]):
            if scores[idx] >= self.min_score:
                scaled_box = [x * width if i % 2 else x * height for i, x in enumerate(boxes[idx])]
                if self.look_for == None:
                    image = self._draw_detection(image, scaled_box, classes[idx], scores[idx] * 100.0, scale_factor)
                else:
                    image, obj_roi = self._draw_detection(image, scaled_box, classes[idx], scores[idx] * 100.0, scale_factor)
                    if self.first_one == 1:
                        if len(obj_roi) > 1:                                                        # exit when we get the first one
                            break
                    else:
                         obj_rois.append(obj_roi)

        if self.look_for == None:
            return image
        else:
            if self.first_one == 1:
                return image, obj_roi
            else:
                return image, obj_rois

    def detect(self, image, display=False):
        im = cv2.resize(image, (640, 640))
        im_np = im

        input_data = {self.input_vstream_info.name: np.array([im_np]).astype(np.float32)}

        network_group_params = self.network_group.create_params()
        input_vstreams_params = InputVStreamParams.make(self.network_group, quantized=False, format_type=FormatType.FLOAT32)
        output_vstreams_params = OutputVStreamParams.make(self.network_group, quantized=True, format_type=FormatType.FLOAT32)

        with InferVStreams(self.network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            with self.network_group.activate(network_group_params):
                infer_results = infer_pipeline.infer(input_data)
                processed_results = self._post_nms_infer(infer_results)
                if self.look_for == None:
                    bbox_image = self._post_process(processed_results, im.copy() if not self.use_pil else im.copy() , 640, 640)
                else:
                    bbox_image, obj_roi = self._post_process(processed_results, im.copy() if not self.use_pil else im.copy() , 640, 640)
        if display:
            if self.use_pil:
                plt.figure(figsize=(6, 6))
                plt.imshow(bbox_image)
                plt.show()
            else:
                cv2.imshow("YOLO Detection", bbox_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        if self.look_for == None:
            return bbox_image
        else:
            return bbox_image, obj_roi

def main():
    hef_path = 'models/yolov10x.hef'                                    # Update with the correct path to your HEF file
    detector = HailoYOLODetector(hef_path)

    try:
        while True:
            image_path = "images/dog.jpg"                               # Update with the correct path to your image file
            image = cv2.imread(image_path)
            bbox_image = detector.detect(image)                         # run detection using the AI accelerator Hailo using YOLO

            cv2.imshow('result', bbox_image)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
      
    except Exception as e:
        print(e)
    finally:
        cv2.destroyAllWindows()

IF_TEST_LIB = True
if __name__ == "__main__":
    if IF_TEST_LIB == True:
        main()
