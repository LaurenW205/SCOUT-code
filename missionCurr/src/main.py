
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import time
import math

def initCam(picam, res): # connect video stream 
    h, w, fps = None, None, None
    if res == 480:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (640, 480)}, controls={"FrameRate": 30})
        picam.configure(config)
        w = 640
        h = 480
        fps = 30.0
    elif res == 720:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1280, 720)}, controls={"FrameRate": 30})
        picam.configure(config)
        w = 1280
        h = 720
        fps = 30.0
    elif res == 1080:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1920, 1080)}, controls={"FrameRate": 30})
        picam.configure(config)
        w = 1920
        h = 1080
        fps = 30.0
        
    picam.start()

    # test video stream connection
    frame = picam.capture_array()
    if frame is None:
        print(f"unable to connect to video stream")
        return 0, picam, w, h, fps
    
    return 1, picam, w, h, fps
         

# Establish connection to video source
picam = Picamera2()
resolution = 720 #480, 720, or 1080p

# Camera initialization
success, picam, frame_W, frame_H, fps = initCam(picam, resolution)

if not success:
    quit(0)

# file output
encoder = H264Encoder(10000000)
output = FfmpegOutput('raw.avi')
picam.start_recording(encoder, output)

# start timer
TIMEOUT = 60 # seconds
start_time = time.time()

while time.time() - start_time < TIMEOUT:
    time.sleep(0.01)

# release camera and data files
picam.stop_recording()
picam.stop()
