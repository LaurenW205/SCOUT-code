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

Pressing R will start/stop recording
Starting/Stopping video multiple times during one session will stitch them together in the output video file.

Pressing I will display the info overlay
This overlay will show current calculated velocity & heading values for a single object moving through the frame

After exiting the camera view, an image marking the path of the object(s) across every frame is shown. There is currently no color distinction between objects.
Again, press Q to exit. (*** IF YOU CLOSE OUT OF THIS WINDOW YOU WILL HAVE TO USE CTRL+C TO FORCIBLY TERMINATE THE PROGRAM)

Velocity calculations using these marked positions will print out in units of pixels per second.

Output Files: rawCap.mp4, processedCap.mp4, TracedImg.png, rawDataOut.txt, and velocityDataOut.txt

## Basic code flow Layout below:

 0. Setup
    - Class and function definitions in external files
    - User defined input settings
       + videoIn - camera index (0 for default, 1 for USB) or file path (e.g. 'rawCap.mp4').
       + blurSize - diameter (in pixels) for with to apply medianblur (*must be an odd integer).
       + binThresh - Pixel intensity threshold (scale 0-255) to decide if pixels are set to black or white.
       + dilIter - number of iterations to dilate a binary frame. Effectivley increases all pixel diameters by 2 px per iteration.
       + minArea - minimum area a contour must have to be considered a trackable object (in pixels)

 1. Initialization
    - Setup camera connection
    - Define output files
    - Define recording & tracking variables

 2. Extract data from frames with openCV library
    - Enter main loop, responsible for recording data and displaying camera feed
    - Apply openCV image processing functions to select contours in each frame
 
 3. Discretize Objects
    - Identify and separate individual objects to track across multiple frames
    - Save location data and populate region-of-interest variables with Node.trackID()
   
 4. Live data logging
    - Save raw node data to files (per node)
    - Save data to frame vars (per node)
    - Save video file output (per frame)

 5. Display window (live)
    - Set display frame (determiend by user key input) & recording dot overlay
    - Display velocity info overlay (for single objects only)

 6. User key input processing
    - Determine display frame
    - Toggle recording & info overlays
    - Exit program 

 7. Post-processing data logging
    - Display traced image and save to file
    - Print terminal output
