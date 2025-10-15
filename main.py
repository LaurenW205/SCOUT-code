## Class and function definitions in external files
import cv2
import numpy as np
from node import Node
import velocityFunc as vf
import time

## Read initial parameters/code settings

# video source (camera index or file path)
videoIn = 0

# blur amount (pixel radius)
blurSize = 5

# binary threshold (0-255 pixel intensity)
binThresh = 25

# dilation amount (number of dilatoin iterations)
dilIter = 2

# min contour area (square pixels)
contArea = 500

## Establish camera connection and initialize out files
capture = cv2.VideoCapture(videoIn)
success, _ = capture.read()
if not success: # check if video source successfully capturing
    print("No Video Source found. \n")
    quit(0)

# Create background subtractor
fgbg = cv2.createBackgroundSubtractorMOG2()

# Define frame dimensions and output file
frame_W = int(capture.get(3))
frame_H = int(capture.get(4))
outfile_mp4 = cv2.VideoWriter('newTestCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))

# Initialize Tracing Image as a black rgb frame
traceImg = np.zeros((frame_H, frame_W, 3), dtype=np.uint8)

recording = False # not recording by default
nodeArray = [] # empty array to store Nodes

# 3.# Enter loop to display camera feed
    # add different keys to change window view (optional)

    # 4.# Apply image filters to find object

    # 5.# Check if recording
        # save tracked object data to array and video file
        # display recording icon in feed

    # 6.# Display video feed


# 7.# Output 
    # image connecting position dots
    # vx, vy velocities
    # V/theta velocities
    # raw recording
    # tracked recording