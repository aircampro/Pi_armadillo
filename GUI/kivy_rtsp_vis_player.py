#!/usr/bin/env python3
#
# Example of using kivy to play a video file chosen, the play stream will be made into a rtsp stream for remote viewing
#
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics.texture import Texture
from kivy.properties import ObjectProperty
from kivy.clock import Clock
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup
import cv2
import time
from ipycam import IPCamera, CameraConfig

Builder.load_file('VideoApp.kv')                                              # load the kivy code

# Video selection pop-up
class LoadDialog(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)

class MyVideoPlayer(BoxLayout):
    image_texture = ObjectProperty(None)
    image_capture = ObjectProperty(None)

    def __init__(self, **kwargs):
        super(MyVideoPlayer, self).__init__(**kwargs)
        self.flagPlay = False                                                # If the video is playing
        self.now_frame = 0                                                   # Variable to check the playback frame of the video for the seek bar
        self.image_index = []                                                # Array to store opencv images for seek bar
        config = CameraConfig.load(                                          # configure streamer
            name="Video Player RTSP output",
            manufacturer="A_C_P",
            model="MyPlayer",
            serial_number="ACME1234",
            # Stream settings
            main_width=1280,
            main_height=720,
            main_fps=30,
            main_bitrate="2M",
            # Substream settings
            sub_width=640,
            sub_height=360,
            sub_bitrate="512K",
            # Hardware acceleration
            hw_accel="auto",                                                  # Options: "auto", "nvenc", "qsv", "cpu"
            # Network ports (optional, uses defaults if not specified)
            onvif_port=8080,
            rtsp_port=8554,
            web_port=8081,
        )
        self.camera = IPCamera(config=config)                                # create remote streamer object
        if not self.camera.start():
            print("Failed to start camera no remote stream available")
        print(f"\n{'='*60}")
        print(f"  IPyCam is running!")
        print(f"{'='*60}")
        print(f"  Web UI:      http://{self.camera.config.local_ip}:{self.camera.config.onvif_port}/")
        print(f"  RTSP Stream: {self.camera.config.main_stream_rtsp}")
        print(f"  ONVIF:       {self.camera.config.onvif_url}")
        print(f"{'='*60}\n")
        self.loaded = False

    def __del__(self):                                                                                   # clean-up and release memory
        if self.loaded == True:
            self.image_capture.release()
            self.loaded = False
        self.camera.stop()

    # select the video file to load 
    def fileSelect(self):
        content = LoadDialog(load = self.load, cancel = self.dismiss_popup)
        self._popup = Popup( title="File Select", content=content, size_hint=(0.9,0.9))
        self._popup.open()

    # load the video file
    def load (self, path, filename):
        txtFName = self.ids['txtFName']
        txtFName.text = filename[0]
        self.image_capture = cv2.VideoCapture(txtFName.text)
        self.loaded = True
        self.sliderSetting()
        self.dismiss_popup()

    def dismiss_popup(self):
        self._popup.dismiss()

    # Setting the Seek Bar
    def sliderSetting(self):
        count = self.image_capture.get(cv2.CAP_PROP_FRAME_COUNT)
        self.ids["timeSlider"].max = count

        # Load the video once and save all the frames to an array
        while True:
            ret, frame = self.image_capture.read()
            self.loaded = ret
            if ret:
                self.image_index.append(frame)
            else:
                self.image_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)                    # re-wind
                break

    # re-wind and play the video
    def play(self):
        self.flagPlay = not self.flagPlay
        if self.flagPlay == True:
            self.image_capture.set(cv2.CAP_PROP_POS_FRAMES, self.now_frame)
            Clock.schedule_interval(self.update, 1.0 / self.image_capture.get(cv2.CAP_PROP_FPS))            # call routine to play video
        else:
            Clock.unschedule(self.update)                                                                   # stop playing by stopping clock scheduler

    # Video playback clock processing
    def update(self, dt):
        ret, frame = self.image_capture.read()                                                              # grab frames
        if ret:                                                                                             # when the next frame is loaded
            self.update_image(frame)                                                                        # display frame on screen
            time = self.image_capture.get(cv2.CAP_PROP_POS_FRAMES)
            self.ids["timeSlider"].value = time
            self.now_frame = int(time)
            self.camera.stream(frame)                                                                       # send frame to rtsp output
    #　Bar
    def siderTouchMove(self):
        Clock.schedule_interval(self.sliderUpdate, 0)

    #　Screen drawing process when the seek bar is moved
    def sliderUpdate(self, dt):
        # When the seek bar value is different from the playback frame value
        if self.now_frame != int(self.ids["timeSlider"].value):
            frame = self.image_index[self.now_frame-1]
            self.update_image(frame)
            self.now_frame = int(self.ids["timeSlider"].value)

    def update_image(self, frame):
        ##################################
        #convert frame into display format
        ##################################
        buf = cv2.flip(frame, 0)
        image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(buf.tostring(), colorfmt='bgr', bufferfmt='ubyte')
        video = self.ids['video']
        video.texture = image_texture

class ShowVideo(App):

    def build(self):
        return MyVideoPlayer()

    def __del__(self):
        del MyVideoPlayer

if __name__ == "__main__":
    try:
        ShowVideo().run()
    except KeyboardInterrupt:
        del ShowVideo
    finally:
        print("video player stream closed")