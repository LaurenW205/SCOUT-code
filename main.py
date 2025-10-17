## Class and function definitions in external files
import cv2
import numpy as np
from node import Node
import velocityFunc as vf
import time

## Read initial parameters/code settings

# video source (camera index or file path, 0 = default camera)
videoIn = 0

# blur amount (pixel radius)
blurSize = 5

# binary threshold (0-255 pixel intensity)
binThresh = 25

# dilation amount (number of dilation iterations)
dilIter = 2

# min contour area (square pixels)
contArea = 500

## Establish camera connection and initialize out files
capture = cv2.VideoCapture(videoIn)
success, _ = capture.read()
if not success: # check if video source successfully capturing
    print("No Video Source found. \n")
    quit(0)

# Create background subtractor [https://docs.opencv.org/3.4/d7/d7b/classcv_1_1BackgroundSubtractorMOG2.html]
fgbg = cv2.createBackgroundSubtractorMOG2()

# Define frame dimensions and output file
frame_W = int(capture.get(3))
frame_H = int(capture.get(4))
outraw_mp4 = cv2.VideoWriter('rawCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))
outproc_mp4 = cv2.VideoWriter('processedCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))

# Initialize Tracing Image as a black rgb frame
traceImg = np.zeros((frame_H, frame_W, 3), dtype=np.uint8)

recording = False # not recording by default
nodeArray = [] # empty array to store Nodes
dispIndex = -1 # tracking frame by default

## Enter loop to display camera feed
while True:
    # Grab frame from video capture
    success, frame = capture.read()
    if not success: # check if successful
        break
    # add different keys to change window view (optional)

    track = frame.copy()

    ## Apply image filters to find object
    # Blur to decrease noise [https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html]
    blur = cv2.medianBlur(frame, blurSize)

    # Apply background subtractor [see link at fgbg definition]
    fgmask = fgbg.apply(blur)

    # Apply Binary threshold at specified intensity (0-255) [https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html]
    _, thresh = cv2.threshold(fgmask, binThresh, 255, cv2.THRESH_BINARY)

    # Dilation (to fill gaps - optional) [https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html]
    dilated = cv2.dilate(thresh, None, iterations=dilIter)

    # Find contours [https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html]
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cont_frame = cv2.drawContours(frame.copy(), contours,  -1,(0,255,0), 3)

    # Loop through possible objects and draw rectangles
    for contour in contours:
        if cv2.contourArea(contour) < contArea: # Filter small contours (square pixel area)
            continue

        # determine bounding rectangle dimensions
        (x, y, w, h) = cv2.boundingRect(contour)

        # draw rectangle
        cv2.rectangle(track, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # check if recording to save specific frame data
        if recording == True:

            # determine centroid
            center_x = x + w//2
            center_y = y + h//2

            # for traced path image, mark centroid dot
            cv2.circle(traceImg, (center_x, center_y), 1, (0, 0, 255), -1)

            # velocity calculation data
            nodeArray.append(Node(center_x, center_y, time.time()))


    ## Check if recording for file output
    if recording == True:
        outraw_mp4.write(frame)
        outproc_mp4.write(track)

        # save tracked object data to array and video file
        # display recording icon in feed

    ## Display video feed
    # set display-frame to track-frame by default
    disp = [frame, blur, fgmask, thresh, dilated, cont_frame, track]
    

    cv2.imshow('Display', disp[dispIndex])

    ## User key input processing
    waitKey = cv2.waitKey(10) & 0xFF #delay in ms between checks
    if waitKey == ord('q'): # quit
        break
    elif waitKey == ord('r'): # toggle recording
        recording = not recording
    elif waitKey== ord('1'): # view raw frame
        dispIndex = 0
    elif waitKey== ord('2'): # view blur
        dispIndex = 1
    elif waitKey== ord('3'): # view foreground mask
        dispIndex = 2
    elif waitKey== ord('4'): # view threshold frame
        dispIndex = 3
    elif waitKey== ord('5'): # view dilated frame
        dispIndex = 4
    elif waitKey== ord('6'): # view contoured frame
        dispIndex = 5
    elif waitKey== ord('7'): # view tracked frame
        dispIndex = 6

# release at termination of video monitor
capture.release()
outraw_mp4.release()
outproc_mp4.release()
cv2.destroyAllWindows()


## Output 
# image connecting position dots
cv2.imshow("Traced Path", traceImg)
while cv2.waitKey(1) != ord('q'):
    pass
cv2.imwrite('TracedImg.png', traceImg)

cv2.destroyAllWindows()
# vx, vy velocities
vx, vy = vf.xyVelocity(nodeArray)

for i in range(len(vx)):
    print(f"X Velocity: {vx[i]}, Y Velocity: {vy[i]}")
# V/theta velocities
# TBD
