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
import glob
import time
import math

def initCam(source): # connect video stream and calibrate lens distortion
    capture = cv2.VideoCapture(source)

    # test video stream connection
    success, _ = capture.read()
    if not success:
        print(f"unable to connect to video stream at source:{source}")
        return 0, capture, None, None
        
    # lens calibration code
    
    #termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
    # prepare object points
    objp = np.zeros((25*25,3), np.float32)
    objp[:,:2] = np.mgrid[0:25, 0:25].T.reshape(-1,2)
    
    # arrays to store points and image points from all images
    objPoints = [] # 3D points in real-world space
    imgPoints = [] # 2D points in image plane
    
    images = glob.glob('photos/*.png')
    shape = None
    for fname in images:
        img = cv2.imread(fname)
        #print(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        shape = gray.shape[::-1]
        
        #find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (25,25), None)
        
        if ret == True:
            #print("check")
            
            objPoints.append(objp)
            
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgPoints.append(corners2)
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, shape, None, None)
    
    return 1, capture, mtx, dist

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

def xyzTransform(x, y, pxSize, trueSize, focalLength, frameDim):
    fx = frameDim[0] * focalLength
    fy = frameDim[1] * focalLength

    z = fx * trueSize / pxSize
    x = (x - (frameDim[0]//2)) * z / fx
    y = (y - (frameDim[1]//2)) * z / fy

    return x, y, z

# camera dimensions [m]
# option 1:
#lensWidth = 0.03
#sensorWidth = 0.008
#focalLength = lensWidth/sensorWidth    
# option 2:   
fieldOfView = 53 # in degrees, adjust for the cropped view after removing distortion
focalLength = 0.5 / (math.tan(fieldOfView * math.pi / 360))

# object real-world size [m]121
targetObjSize = 0.08

# minimum object size detectable in pixel area
minObjSize = 10            

# Camera source (file or index)
videoIn = 0

# global node arrays
currFrameNodes = []
prevFrameNodes = []

# Camera initialization (calibrate distortion)
success, capture, mtx, dist = initCam(videoIn)

if not success:
    quit(0)

frame_W = int(capture.get(3))
frame_H = int(capture.get(4))
dispIndex = 0

# file output
raw_mp4 = cv2.VideoWriter('rawCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))
contour_mp4 = cv2.VideoWriter('contourCap.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_W,frame_H))

while True:
    success, frame = capture.read()
    if not success: # check if successful
        break

    ncmtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (frame_W,frame_H), 0, (frame_W,frame_H))
    #mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, ncmtx, (frame_W,frame_H), 5)
    #undistorted = cv2.remap(filtered, mapx, mapy, cv2.INTER_LINEAR)
    undistorted = cv2.undistort(frame, mtx, dist, None, ncmtx)
    
    cropped = undistorted[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    
    filtered = filterRGB(2, cropped, 50, 20) 

    contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cont_frame = cropped.copy()

    # loop through all objects in current frame
    prevFrameNodes = currFrameNodes
    currFrameNodes = []
    for contour in contours:

        # Filter out small contours (square pixel area)
        if cv2.contourArea(contour) < minObjSize: 
            continue

        cont_frame = cv2.drawContours(cont_frame, contour,  -1, (0,255,0), 3)

        # determine bounding rectangle dimensions
        (x, y, w, h) = cv2.boundingRect(contour)

        # determine centroid, size, & current time
        c_x = x + w//2
        c_y = y + h//2
        unitLength = max(w, h)
        currTime = time.time()

        
        # estimate object position in cartesian coordinates
        (x, y, z) = xyzTransform(c_x, c_y, unitLength, targetObjSize, focalLength, (frame_W, frame_H))

        # save object position
        obj = [x, y, z, currTime]
        print(obj)
        currFrameNodes.append(obj)

    # initialize array of video streams (for 1-7 key functionality)
    dispArr = [frame, undistorted, cropped, filtered, cont_frame]
    disp = dispArr[dispIndex].copy()

    # write to file
    raw_mp4.write(frame)
    contour_mp4.write(disp)

    # Display video feed
    cv2.imshow('Display', disp)

    waitKey = cv2.waitKey(16) & 0xFF #delay in ms between checks = 16
    if waitKey == ord('q'): # quit
        break
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


capture.release()
raw_mp4.release()
contour_mp4.release()
cv2.destroyAllWindows()
