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
import subprocess
import time
import math
import node
import kalman

def initCam(picam, res): # connect video stream 
    h, w = None, None
    if res == 480:
        #subprocess.run(["./setCamResolution/480.sh"])
        config = picam.create_video_configuration({"format" : "RGB888", "size": (640, 480)})
        picam.configure(config)
        h = 640
        w = 480
    elif res == 720:
        #subprocess.run(["./setCamResolution/720.sh"])
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1280, 720)})
        picam.configure(config)
        h = 1280
        w = 720
    elif res == 1080:
        #subprocess.run(["./setCamResolution/1080.sh"])
        config = picam.create_video_configuration({"format" : "RGB888", "size": (1920, 1080)})
        picam.configure(config)
        h = 1920
        w = 1080
        
    picam.start()

    # test video stream connection
    frame = picam.capture_array()
    if frame is None:
        print(f"unable to connect to video stream at source:{source}")
        return 0, picam, w, h, None, None
        
        if needCalibrate is True:
            mtx, dist = calibrate(res)
            return 1, picam, w, h, mtx, dist
    
    return 1, picam, w, h, None, None

def calibrate(res): # calibrate lens distortion
    # LENS CALIBRATION
    
    #termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
    # prepare object points
    objp = np.zeros((25*25,3), np.float32)
    objp[:,:2] = np.mgrid[0:25, 0:25].T.reshape(-1,2)
    
    # arrays to store points and image points from all images
    objPoints = [] # 3D points in real-world space
    imgPoints = [] # 2D points in image plane
    
    images = glob.glob(f"photos{res}/*.png")
    shape = None
    totImgCount = 0
    sucImgCount = 0
    for fname in images:
        totImgCount += 1
        
        img = cv2.imread(fname)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        shape = gray.shape[::-1]
        
        #find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (25,25), None)
        
        if ret == True:
            sucImgCount += 1
            
            objPoints.append(objp)
            
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgPoints.append(corners2)
        else:
            print(f"{fname}") # prints filenames of unused images
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, shape, None, None)
    return mtx, dist
    
    
def filterRGB(color_idx, frame, minThresh, maxDiff):
    # Create a mask to filter out pixels where the target color is too dim
    low_intensity = frame[:, :, color_idx] < minThresh
    
    # Create a mask to filter out pixels where other colors are too strong 
    # (Checking all channels except the target 'color_idx')
    too_different = np.zeros(frame.shape[:2], dtype=bool)
    for i in range(3):
        if i != color_idx:
            # Mask is True if (OtherChannel - TargetChannel) > maxDiff
            too_different |= (frame[:, :, i].astype(int) - frame[:, :, color_idx].astype(int)) > maxDiff

    # Combine masks: any pixel matching either condition gets zeroed out
    mask = low_intensity | too_different
    
    filtered = frame.copy()
    filtered[mask] = [0, 0, 0]  # Black out the pixels
    
    # isolate target color
    for i in range(3):
        if i != color_idx:
            filtered[:, :, i] = 0

    # make monochrome (for contouring)
    filtered = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

    # Apply Binary threshold at specified intensity (0-255) [https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html]
    _, filtered = cv2.threshold(filtered, 1, 255, cv2.THRESH_BINARY)
    
    # invert black and white for contouring
    #filtered = cv2.bitwise_not(filtered)
            
    return filtered
    
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
    
    # invert black and white for contouring
    #filtered = cv2.bitwise_not(filtered)
            
    return filtered

def xyzTransform(x, y, pxSize, trueSize, focalLength, frameDim):
    fx = frameDim[0] * focalLength
    fy = frameDim[1] * focalLength
    #fx = ncmtx[0, 0]
    #fy = ncmtx[1, 1]
    #cx = ncmtx[0, 2]
    #cy = ncmtx[1, 2]

    z = fx * trueSize / pxSize
    x = (x - (frameDim[0]//2)) * z / fx
    y = (y - (frameDim[1]//2)) * z / fy
    
    #x = (x - cx) * z / fx
    #y = (y - cy) * z / fy

    return x, y, z

# camera dimensions [m]
fieldOfView = 75 #53 # in degrees, adjust for the cropped view after removing distortion
focalLength = 0.5 / (math.tan(fieldOfView * math.pi / 360))

# need undistort?
needCalibrate = False

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
success, picam, frame_W, frame_H, mtx, dist = initCam(picam, resolution)

if not success:
    quit(0)


dispIndex = 0

# file output
encoder = H264Encoder(10000000)
output = FfmpegOutput('raw.mp4')
picam.start_recording(encoder, output)
#raw_mp4 = cv2.VideoWriter('rawCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))
#contour_mp4 = cv2.VideoWriter('contourCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))
dataFile = open("data.txt", 'w')
dataFile.write(f"id, x, y, z, t, mx, my, mz \n")

# object node counter (for unique node ids)
nodeCount = 0

# calculate matrix to undistort camera image
if needCalibrate is True:
    ncmtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (frame_W,frame_H), 0, (frame_W,frame_H))

# array of colors for tracking frame boxes  red, orange, yellow, green, blue, purple, pink
COLORS = [(0,0,255),(0,127,255),(0,255,255),(0,255,0),(255,0,0),(127,0,127),(191,191,255)]

saveData = False

while True:
    frame = picam.capture_array()
    if frame is None: # check if successful
        break
        
    # default w/out calibration
    cropped = frame
    undistorted = cropped
    
    if needCalibrate is True:
        undistorted = cv2.undistort(frame, mtx, dist, None, ncmtx)
        cropped = undistorted #undistorted[int(0.2*frame_H):int(0.8*frame_H),int(0.4*frame_W):int(0.6*frame_W)]
    
    #filtered = filterRGB(2, cropped, 35, 20) 
    filtered = filterYRB(cropped, 70, 165, 20) 

    contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cont_frame = cropped.copy()
    track = cropped.copy()

    # loop through all objects in current frame
    prevFrameNodes = currFrameNodes
    currFrameNodes = []
    for contour in contours:

        # Filter out small contours (square pixel area)
        if cv2.contourArea(contour) < minObjSize: 
            continue

        cont_frame = cv2.drawContours(cont_frame, contour,  -1, (0,255,0), 3)

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
        #print(f"id: {obj.id}, x: {obj.x}, y: {obj.y}, z: {obj.z} \n")
        if saveData == True:
            dataFile.write(f"{obj.id}, {obj.x}, {obj.y}, {obj.z}, {currTime}, {obj.mx}, {obj.my}, {obj.mz} \n")
            print(f"id: {obj.id}, x: {obj.x}, y: {obj.y}, z: {obj.z} \n")
            

        # if object is a new object, increment
        if initID == obj.id:
            nodeCount += 1
            
        # update tracking frame
        colorIdx = obj.id % 7
        color = COLORS[colorIdx]
        cv2.rectangle(track, (rx, ry), (rx + w, ry + h), color, 2)

    # initialize array of video streams (for 1-7 key functionality)
    dispArr = [frame, undistorted, cropped, filtered, cont_frame, track]
    disp = dispArr[dispIndex].copy()

    # write to file (not needed for picam)
    #raw_mp4.write(frame)
    #contour_mp4.write(disp)

    # Display video feed
    cv2.imshow('Display', disp)

    waitKey = cv2.waitKey(16) & 0xFF #delay in ms between checks = 16
    if waitKey == ord('q'): # quit
        break
    elif waitKey== ord('s'): # save current node
        saveData = not saveData
        if saveData:
            print("recording...")
        else:
            print("recording stopped.")
    elif waitKey== ord('1'): # view raw frame
        dispIndex = 0
    elif waitKey== ord('2'): # view filtered frame
        dispIndex = 1
    elif waitKey== ord('3'): # view undistorted frame
        dispIndex = 2
    elif waitKey== ord('4'): # view cropped frame
        dispIndex = 3
    elif waitKey== ord('5'): # view contours on frame
        dispIndex = 4
    elif waitKey== ord('6'): # view contours on frame
        dispIndex = 5


picam.stop_recording()
#raw_mp4.release()
#contour_mp4.release()
dataFile.close()
picam.stop()
cv2.destroyAllWindows()

