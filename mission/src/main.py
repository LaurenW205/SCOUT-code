# Detect red pixels in frame DONE
    # filter R > minThresh
    # filter BG < maxDiff

# estimate object size DONE
    # contour filtered image
    # transform contour area -> pixel length
        # account for various orientations of LunaSat

# add debug ui DONE

# estimate object distance
    # transform pixel coordinates to XYZ DONE
    # correct for lens distortion

# track objects across frames based on position
    # basic kalman filter to estimate next position
    # weighted selection matrix 

# estimate object telemtery: pos, vel, acc

import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import glob
import time
import math
import node
import kalman

def initCam(picam, res): # connect video stream 
    h, w = None, None
    if res == 480:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (640, 480)})
        picam.configure(config)
        h = 640
        w = 480
    elif res == 720:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1280, 720)})
        picam.configure(config)
        h = 1280
        w = 720
    elif res == 1080:
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1920, 1080)})
        picam.configure(config)
        h = 1920
        w = 1080
        
    picam.start()

    # test video stream connection
    frame = picam.capture_array()
    if frame is None:
        print(f"unable to connect to video stream")
        return 0, picam, w, h
    
    return 1, picam, w, h

def filterYRB(frame, lumThresh, redThresh, maxDiff):
    # Convert frame to YCrCb
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    
    # Create a mask to filter out pixels where red is too dim
    low_red = frame[:, :, 1] < redThresh 
    
    # Create a mask to filter out pixels where blue is too strong 
    too_different = np.zeros(frame.shape[:2], dtype=bool)
    
    # Mask is True if (BlueChannel - RedChannel) > maxDiff
    too_different |= (frame[:, :, 2].astype(int) - frame[:, :, 1].astype(int)) > maxDiff

    # Combine masks: any pixel matching either condition gets zeroed out
    mask = low_red | too_different
    
    filtered = frame.copy()
    filtered[mask] = [0, 0, 0]  # Black out the pixels

    # make monochrome (for contouring)
    filtered = filtered[:, :, 0]

    # Apply Binary threshold at specified intensity (0-255) [https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html]
    _, filtered = cv2.threshold(filtered, lumThresh, 255, cv2.THRESH_BINARY)
    
    # invert black and white
    #filtered = cv2.bitwise_not(filtered)
            
    return filtered

def xyzTransform(x, y, pxSize, trueSize, focalLength, frameDim):
    fx = frameDim[0] * focalLength
    fy = frameDim[1] * focalLength

    z = fx * trueSize / pxSize
    x = (x - (frameDim[0]//2)) * z / fx
    y = (y - (frameDim[1]//2)) * z / fy

    return x, y, z

# camera dimensions [m]
fieldOfView = 75 # in degrees
focalLength = 0.5 / (math.tan(fieldOfView * math.pi / 360))

# object real-world size [m]121
targetObjSize = 0.08

# minimum object size detectable in pixel area
minObjSize = 5            

# Establish connection to video source
picam = Picamera2()
resolution = 720 #480, 720, or 1080p

# global node arrays
currFrameNodes = []
prevFrameNodes = []

# Camera initialization (calibrate distortion)
success, picam, frame_W, frame_H = initCam(picam, resolution)

if not success:
    quit(0)

# file output
encoder = H264Encoder(10000000)
output = FfmpegOutput('raw.mp4')
picam.start_recording(encoder, output)
dataFile = open("data.txt", 'w')
dataFile.write(f"id, x, y, z, t, mx, my, mz \n")

# object node counter (for unique node ids)
nodeCount = 0

# start timer
TIMEOUT = 60 # seconds
start_time = time.time()

while True:
    frame = picam.capture_array()
    if frame is None: # check if successful
        break
        
    # default w/out calibration
    cropped = frame
    undistorted = cropped
    
    filtered = filterYRB(cropped, 70, 165, 20) 

    contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # loop through all objects in current frame
    prevFrameNodes = currFrameNodes
    currFrameNodes = []
    for contour in contours:

        # Filter out small contours (square pixel area)
        if cv2.contourArea(contour) < minObjSize: 
            continue

        # determine bounding rectangle dimensions
        (rx, ry, w, h) = cv2.boundingRect(contour)

        # determine centroid, size, & current time
        c_x = rx + w//2
        c_y = ry + h//2
        unitLength = max(w, h)
        currTime = time.time()

        # estimate object position in cartesian coordinates
        (x, y, z) = xyzTransform(c_x, c_y, unitLength, targetObjSize, focalLength, (frame_W, frame_H))

        # save object position
        obj = node.Node(nodeCount, (x, y, z), currTime)
        initID = obj.id

        # populate/update remaining node data
        obj = kalman.kalmanFilter(obj, prevFrameNodes)

        # save to array
        currFrameNodes.append(obj)
        
        dataFile.write(f"{obj.id}, {obj.x}, {obj.y}, {obj.z}, {currTime}, {obj.mx}, {obj.my}, {obj.mz} \n")            

        # if object is a new object, increment
        if initID == obj.id:
            nodeCount += 1
            

    if time.time() - start_time > TIMEOUT:
        break

# release camera and data files
picam.stop_recording()
dataFile.close()
picam.stop()
