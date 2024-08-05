#!/usr/bin/env python
# Capture a camera and publish it and messages to ROS2
#
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import time
import os
import glob

# Capture a camera and publish to ROS2 channel and save to disk 
class CaptureCameraNode(Node):
    def __init__(self):
        # Инициализируем родительский класс Node с именем 'capture_camera'
        super().__init__('capture_camera')
        
        # Создаем объект CvBridge для преобразования изображений между OpenCV и ROS
        self.bridge = CvBridge()

        # Создаем издателей для публикации изображений и текстовых сообщений
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.text_publisher = self.create_publisher(String, 'camera/info', 10)
        self.number_pics = self.create_publisher(String, 'camera/pic_counter', 10)
        self.pic_counter = 0
        
        # Создаем таймер, который вызывает функцию capture_and_publish каждую секунду
        self.timer = self.create_timer(1.0, self.capture_and_publish)

        # Определяем путь для сохранения изображений
        self.image_save_path = "/home/pi/images"
        # Если папка для сохранения изображений не существует, создаем ее
        os.makedirs(self.image_save_path, exist_ok=True)
        
        # Создаем таймер, который вызывает функцию save_image каждые 5 секунд
        self.save_timer = self.create_timer(5.0, self.save_image)

    # set the camera type
    # 0 = usb cam
    # 1 = Thermal FLIR Lepcon 3.5
    #
    def set_cam_type(self, ct=0):
        self.cam_type = ct
        if self.cam_type == 0:
            # path to the capture device
            path = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0"
            # set the capture device
            self.cap = cv2.VideoCapture(path, cv2.CAP_V4L)
            print("Camera created", self.cap) 
        elif self.cam_type == 1:
            from thermal_camera_init import *
            from uvctypes import *
            self.cap = ThermalCamera(0, 0)                             # thermal camera with format UVC_FRAME_FORMAT_Y16 & VS_FMT_GUID_Y16
            print("Camera created", self.cap) 
            
    # Capture the camera frame and publish it to ROS2
    #
    def capture_and_publish(self):
        if self.cam_type == 0:
            # read the usb camera
            ret, frame = self.cap.read()
            if ret:
                # show the image
                print("Last time of get image:", time.ctime())
                cv2.imshow("usb cam image", frame)
               
                # publish the image
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher.publish(msg)

                # publish string with time and a counter and camera type message 
                msg = String()
                msg.data = f"Last time of get image: {time.ctime()}"
                self.text_publisher.publish(msg)

                self.pic_counter += 1
                msg = String()
                msg.data = 'Number of pictures taken %d camera type %d' % (self.pic_counter, self.cam_type)
                self.number_pics.publish(msg)
        elif self.cam_type == 1:
            # read the thermal FLIR Lepcon 3.5
            for thermal_frame in self.cap.read_thermal_data():
                # show the image
                print(thermal_frame.shape)
                radiometric_image = self.cap.raw_to_8bit(thermal_frame)
                cv2.imshow("FLIR radio metric image", radiometric_image)
                
                # publish the image                
                #msg = self.bridge.cv2_to_imgmsg(thermal_frame, "bgr8")
                msg = self.bridge.cv2_to_imgmsg(radiometric_image, "bgr8")
                self.publisher.publish(msg)       

                # publish string with time and a counter and camera type message 
                msg = String()
                msg.data = f"Last time of get image: {time.ctime()}"
                self.text_publisher.publish(msg)

                self.pic_counter += 1
                msg = String()
                msg.data = 'Number of pictures taken %d camera type %d' % (self.pic_counter, self.cam_type)
                self.number_pics.publish(msg)
                
    # save the camera image to a timestamped file name
    def save_image(self):
        if self.cam_type == 0:
            ret, frame = self.cap.read()
            if ret:
                # get current timestamp and write the file
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(self.image_save_path, f"image_{timestamp}.jpg")
                cv2.imwrite(filename, frame)
                print(f"Saved image to {filename}")
            
                # Получаем список всех сохраненных изображений
                all_images = glob.glob(os.path.join(self.image_save_path, "*.jpg"))
                # Сортируем список изображений по дате создания
                sorted_images = sorted(all_images, key=os.path.getmtime)
                # Удаляем старые изображения, оставляя только 20 последних
                while len(sorted_images) > 20:
                    os.remove(sorted_images[0])
                    del sorted_images[0]
        elif self.cam_type == 1:
            # read the thermal FLIR Lepcon 3.5
            for thermal_frame in self.cap.read_thermal_data():
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(self.image_save_path, f"image_{timestamp}.jpg")
                #cv2.imwrite(filename, thermal_frame)
                radiometric_image = self.cap.raw_to_8bit(thermal_frame)
                cv2.imwrite(filename, radiometric_image)
                print(f"Saved image to {filename}")       

                # Получаем список всех сохраненных изображений
                all_images = glob.glob(os.path.join(self.image_save_path, "*.jpg"))
                # Сортируем список изображений по дате создания
                sorted_images = sorted(all_images, key=os.path.getmtime)
                # Удаляем старые изображения, оставляя только 20 последних
                while len(sorted_images) > 20:
                    os.remove(sorted_images[0])
                    del sorted_images[0]

def main(args=None):
    # Инициализируем ROS
    rclpy.init(args=args)
    # Создаем объект нашей ноды
    capture_camera = CaptureCameraNode()
    capture_camera.set_cam_type(1)                        # set FLIR camera to be the camera used for cv cam on usb change to 0
    # Запускаем цикл обработки ROS
    rclpy.spin(capture_camera)

if __name__ == '__main__':
    main()