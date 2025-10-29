### 0 ### SETUP
##########################################################
## 0.1 ## Class and function definitions in external files

import cv2
import numpy as np
from node import Node
import velocityFunc as vf
import time
import math

##############################################################
## 0.2 ## Read initial parameters/code settings - USER DEFINED

# video source (camera index or file path, 0 = default camera)
videoIn = 0

# blur amount (pixel radius)
blurSize = 5

# binary threshold (0-255 pixel intensity)
binThresh = 25

# dilation amount (number of dilation iterations)
dilIter = 2

# min contour area (square pixels)
minArea = 500

### 1 ### Initialization
#####################################
## 1.1 ## Establish camera connection 
capture = cv2.VideoCapture(videoIn)
success, _ = capture.read()
if not success: # check if video source successfully capturing
    print("No Video Source found. \n")
    quit(0)

#################################
## 1.2 ## Initialize output files 

# Define frame dimensions and video output files
frame_W = int(capture.get(3))
frame_H = int(capture.get(4))
outraw_mp4 = cv2.VideoWriter('rawCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))
outproc_mp4 = cv2.VideoWriter('processedCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))

# Initialize Tracing Image as a black rgb frame
traceImg = np.zeros((frame_H, frame_W, 3), dtype=np.uint8)

# Initialize Data Files (comma separated list)
rawData = open('rawDataOut.txt', 'w')
rawData.write("id, x, y, t,\n")
velData = open('velocityDataOut.txt', 'w')
velData.write("id, vx, vy, v, theta,\n")

###################################################
## 1.3 ## Initialize recording & tracking variables

# bool to determine when saving data vs not
recording = False # not recording by default

# stores current recorded frame num, for node data
recFrameCount = 0 # current frame count of recorded frames

# current number of object IDs (tracked objects)
idCount = 0 

# bool determining if info display is shown
info = False # info display off by default

# empty array to store all Nodes (data not saved) for info display
allNodes = [] 

# empty array to store recorded Nodes
recNodes = [] 

# index to choose displayed frame (to user) using keys 1-7
dispIndex = -1 # tracking frame by default

# Create background subtractor [https://docs.opencv.org/3.4/d7/d7b/classcv_1_1BackgroundSubtractorMOG2.html]
fgbg = cv2.createBackgroundSubtractorMOG2()

### 2 ### Extract data from frames (openCV)
####################################################
## 2.1 ## Enter loop to display & record camera feed

while True:
    # Grab frame from video capture
    success, frame = capture.read()
    if not success: # check if successful
        break

    # separate video frame-stream saved to outproc_mp4 file, draws tracking boxes on video
    track = frame.copy() 

    #####################################################
    ## 2.2 ## Apply image filters to find object contours

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

    # create frame with contours drawn as a display option (keys 1-7)
    cont_frame = cv2.drawContours(frame.copy(), contours,  -1, (0,255,0), 3)

    ### Discretize objects
    #########################################################
    ## 3.1 ## Track objects across frames

    # retrieve nodes from prior objects for 'id' tracking
    prevNodes = Node.getPrevNodes(recNodes)

    # Loop through possible objects and draw rectangles
    for contour in contours:

        # Filter out small contours (square pixel area)
        if cv2.contourArea(contour) < minArea: 
            continue

        # determine bounding rectangle dimensions
        (x, y, w, h) = cv2.boundingRect(contour)

        # determine centroid & current time
        center_x = x + w//2
        center_y = y + h//2
        curr_time = time.time()

        #############################################
        ## 3.2 ## Populate node position and ROI data

        # check if recording to save specific frame data
        if recording == True:
            # update frame count
            recFrameCount = recFrameCount + 1

            # velocity calculation data 
            recNode = Node(center_x, center_y, curr_time, recFrameCount)

            # set new region of interest and update idCount
            idCount = recNode.trackID(prevNodes, idCount, (frame_W, frame_H))

            # save to array
            recNodes.append(recNode)

            ### 4 ### Live data logging
            ###########################
            ## 4.1 ## Save raw node data to file

            # for traced-path image, mark centroid dot 
            cv2.circle(traceImg, (center_x, center_y), 1, (0, 0, 255), -1)

            # save raw data to file
            rawData.write(f"{recNode.id}, {center_x}, {center_y}, {curr_time},\n")

        ## 4.2 ## Save frame data

        # draw rectangle to track frame 
        cv2.rectangle(track, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # save data to allNodes, distinguish from recorded data by frame_num = -1 
        allNodes.append(Node(center_x, center_y, curr_time, -1))




    ## 4.3 ## Check if recording for video file output
    if recording == True:

        # save tracked object data to video file
        outraw_mp4.write(frame)
        outproc_mp4.write(track)

    ### 5 ### Display window (live)

    ###################################################
    ## 5.1 ## set display frame & recording dot overlay

    # initialize array of video streams (for 1-7 key functionality)
    dispArr = [frame, blur, fgmask, thresh, dilated, cont_frame, track]
    disp = dispArr[dispIndex].copy()
    
    # display recording icon in feed
    if recording == True: 
        cv2.circle(disp, (50, 50), 15, (0, 0, 255), -1)
    
    #############################
    ## 5.2 ## display info screen
    if info == True:
        cv2.putText(disp, "I", (frame_W-70, 70),cv2.FONT_HERSHEY_COMPLEX,2,(240,180,50),2)
        if len(allNodes) > 2: # check if at startup (empty array)

            # calculate values at most recent node data
            vxi, vyi = vf.xyVelocity(allNodes[-2:])
            vi, thetai = vf.vthetaVelocity(allNodes[-2:])

            # display info on screen
            cv2.putText(disp, f"Vel x : {math.floor(vxi[0])}", (20,frame_H-50),cv2.FONT_HERSHEY_PLAIN,1.2,(240,180,50),2)
            cv2.putText(disp, f"Vel y : {math.floor(vyi[0])}", (160,frame_H-50),cv2.FONT_HERSHEY_PLAIN,1.2,(240,180,50),2)
            cv2.putText(disp, f"Tot V : {math.floor(vi[0])}", (20,frame_H-20),cv2.FONT_HERSHEY_PLAIN,1.2,(240,180,50),2)
            cv2.putText(disp, f"Theta : {math.floor(thetai[0])}", (160,frame_H-20),cv2.FONT_HERSHEY_PLAIN,1.2,(240,180,50),2)

    # Display video feed
    cv2.imshow('Display', disp)


    ### 6 ### User key input processing
    #####################################################
    ## 6.1 ## set frame, toggle overlays, or exit program

    waitKey = cv2.waitKey(10) & 0xFF #delay in ms between checks
    if waitKey == ord('q'): # quit
        break
    elif waitKey == ord('r'): # toggle recording
        recording = not recording
    elif waitKey == ord('i'): # toggle info display
        info = not info
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

# release media resources at termination of video monitor
capture.release()
outraw_mp4.release()
outproc_mp4.release()
cv2.destroyAllWindows()


### 7 ### Post-processing data logging 
#######################################
## 7.1 ## traced image display and file

# display img
cv2.imshow("Traced Path", traceImg)
while cv2.waitKey(1) != ord('q'):
    pass

# save traced image to file
cv2.imwrite('TracedImg.png', traceImg)

# close window
cv2.destroyAllWindows()

#################################
## 7.2 ## Terminal printed output

# print velocity data to file and terminal
for j in range(idCount):
    # correct index '0 to (idCount-1)' into '1 to idCount'
    j = j + 1

    # extract nodes by ID
    currNodes = Node.getNodesByID(recNodes, j)

    # vx, vy velocities
    vx, vy = vf.xyVelocity(currNodes)

    # V/theta velocities
    v, theta = vf.vthetaVelocity(currNodes)

    # print relevant id (terminal only)
    print(f"Object ID: {j}")
    # loop through velocity values
    for i in range(len(vx)):
        velData.write(f"{j}, {vx[i]}, {vy[i]}, {v[i]}, {theta[i]},\n")
        print(f"     X Velocity: {vx[i]}, Y Velocity: {vy[i]}, Total Velocity: {v[i]}, Heading Angle (deg): {theta[i]}")

# release data files
rawData.close()
velData.close()
