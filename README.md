# SCOUT-code
## Main File UI instructions

After running the code, a window will pop up showing your camera's view, and likely, some tracking boxes.
To exit this window and any subsequent windows, press Q at any time.

You can cycle through the camera view options with keys 1-7. The views are as follows:
1. Raw Camera Frame
2. Blurred Camera Frame
3. Foreground Mask Frame
4. Binary Threshold frame
5. Dilated Frame
6. Contoured Frame
7. Tracking Frame

Pressing R will start recording (no indicator as of yet)
Pressing R will also stop recording (again no indicator)

Starting/Stopping video multiple times during one session will stitch them together in the output video file.

After exiting the camera view, an image marking the path of the object across every frame is shown. Only one object can be tracked with this method at this time.
Again, press Q to exit.

Velocity calculations using these marked positions will print out in units of pixels per second.

Output Files: rawCap.mp4, processedCap.mp4, and TracedImg.png 

## Basic code flow Layout below:

 0. Class and function definitions in external files

 1. Read initial parameters/code settings
    - FGBG mask
    - blur amount
    - dilation amount
    - min contour size

 2. Establish camera connection and initialize out files

 
 3. Enter loop to display camera feed
    - add different keys to change window view (optional)

 5. Apply image filters to find object

 6. Check if recording
        - save tracked object data to array and video file
        - display recording icon in feed

 7. Display video feed, Exit loop

 8. Output 
    - image connecting position dots
    - vx, vy velocities
    - V/theta velocities
    - raw recording
    - tracked recording
