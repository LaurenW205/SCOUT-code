#!/bin/bash

### Drives mission code


## Boot Up & Initialization
# this is for when the code is run manually for debug
if [[ $1 -gt 0 ]]; then 
    # skip unnecessary stuff, run main loop number of times as in $1 ------------------------------
else
    # wait 3 seconds for to allow for Ctrl+C interrupt
    sleep 3
fi

LOGFILE="mission.log"
MAINFILE="run/main.exe"

# create .log file
echo -n "" > $LOGFILE
echo "Log file created at $(date)" >> $LOGFILE

# check memory allocation
FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
echo "Remaining Free Memory: $FREEMEM MB" >> $LOGFILE

# idle and wait for interrupt?
trap break INT
echo "Idling until interrupt..." >> $LOGFILE

while true; do
    sleep 1
done

echo "Interrupt received | $(date)" >> $LOGFILE

## Main Loop
trap 'exit 0' INT

LOOPCOUNT=1
while :; do

    # For debug loop termination
    if [[ -n $1 ]] && [[ $LOOPCOUNT -eq $1 ]]; then
        break
    fi

    echo "Loop: $LOOPCOUNT" >> $LOGFILE

# 1. Check available memory storage
    FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
    echo "Remaining Free Memory: $FREEMEM MB" >> $LOGFILE

    # if remaining storage is < 100MB
    if [[ $FREEMEM -lt 100 ]]; then
        # 2 oldest data files
        read -r FILE1 FILE2 < <(ls -tr1 data/ | head -n 2)
        # delete oldest files (-f to force)
        echo "Removing $FILE1, $FILE2" >> $LOGFILE
        rm "data/$FILE1" "data/$FILE2" -f
    fi

# 2. Call run/ files

    # run main executable in /run --- terminate after 1 min 10 sec
    timeout -k 10s 1m ./$MAINFILE >> $LOGFILE
        # if error, recompile from /src?


# 3. Organize Data Output

    # move and rename data files
    echo "Catalogging data files" >> $LOGFILE
    mv data.txt data/data$LOOPCOUNT.txt
    mv raw.mp4 data/raw$LOOPCOUNT.mp4

    LOOPCOUNT=$((LOOPCOUNT + 1))

done