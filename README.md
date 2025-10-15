# SCOUT-code

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
