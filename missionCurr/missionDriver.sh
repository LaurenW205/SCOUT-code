#!/bin/bash

### Drives mission code


## Boot Up & Initialization

cd /home/scout || exit

BASEDIR="Desktop/mission"

# create .log file
LOGFILE="mission.log"
echo -n "" > "${BASEDIR}/${LOGFILE}"
sleep 1
echo "Log file created at $(date)" >> "${BASEDIR}/${LOGFILE}"

sudo echo 536 > sys/class/gpio/export
sleep 1
sudo echo in > sys/class/gpio/gpio536/direction
echo "GPIO initialized" >> "${BASEDIR}/${LOGFILE}"

# check for residual data file
RESFILE=$(ls -1 "${BASEDIR}" -- *.h264 2>"${BASEDIR}/${LOGFILE}" | head -n 1 )
OLDTIME=$(date)
if [[ -n $RESFILE ]]; then
    # extract previous timestamp
    OLDFILE=$(ls -1t -- "${BASEDIR}/data/*raw*.h264" 2>"${BASEDIR}/${LOGFILE}" | head -n 1 )

    # if none found, set to empty
    if [[ -z "$OLDFILE" ]]; then
        echo "No old files found." >> "${BASEDIR}/${LOGFILE}"
    else
        # trim the suffix "raw*.h264" from the end of the filename
        OLDTIME="${OLDFILE%raw*.h264}"
        # only want the basename (no directory)
        OLDTIME="$(basename -- "$OLDTIME")"
    fi
    
    mv -- "$RESFILE" "${BASEDIR}/data/$OLDTIME$RESFILE"
    echo "Moved: $RESFILE -> data/" >> "${BASEDIR}/${LOGFILE}"
else
    echo "No residual .h264 files found." >> "${BASEDIR}/${LOGFILE}"
fi

MAINFILE="src/main.py"
VFILETYPE="h264"

# check memory allocation
FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
echo "Remaining Free Memory: $FREEMEM MB" >> "${BASEDIR}/${LOGFILE}"

# check current number of files to determine file index later on
FILECOUNT=$(find "${BASEDIR}/data" -maxdepth 1 -type f -name "*.${VFILETYPE}" 2>"${BASEDIR}/${LOGFILE}" | wc -l)

HOST_IP="10.12.194.1"
echo "Waiting for Pi 4 (Host) to wake up..." >> "${BASEDIR}/${LOGFILE}"
# Loop until we can successfully ping the Pi 4
until ping -c 1 -W 1 "$HOST_IP" >> "${BASEDIR}/${LOGFILE}" 2>&1; do
    sleep 1
done
MAX_PING_ATTEMPTS=30
ping_attempts=0

until ping -c 1 -W 1 "$HOST_IP" >> "${BASEDIR}/${LOGFILE}" 2>&1; do
    ((ping_attempts++))
    echo "Ping attempt $ping_attempts failed" >> "${BASEDIR}/${LOGFILE}"
    if (( ping_attempts >= MAX_PING_ATTEMPTS )); then
        echo "Host $HOST_IP did not respond after $ping_attempts attempts." >> "${BASEDIR}/${LOGFILE}"
        exit 1
    fi
    sleep 1
done

if ((ping_attempts < MAX_PING_ATTEMPTS)); then
    echo "Pi 4 found. Pulling system time..." >> "${BASEDIR}/${LOGFILE}"
    # Pull the time from the Pi 4 and set it locally
    # -o ConnectTimeout=5 ensures it doesn't hang if the link drops
    HOST_TIME=$(ssh -o ConnectTimeout=5 rocksat@$HOST_IP "date -uR")

    if (($? == 0)); then
        sudo date -s "$HOST_TIME"
        echo "Time synced: $(date)" >> "${BASEDIR}/${LOGFILE}"
    else
        echo "Failed to pull time from Pi 4." >> "${BASEDIR}/${LOGFILE}"
        echo "Timestamp utilized will be $(date)" >> "${BASEDIR}/${LOGFILE}"
        # Optional: Add error handling or exit here
    fi
fi

# Idle until LOW signal received on GPIO 24
while :; do
    SIGNAL=$(cat sys/class/gpio/gpio536/value)
    if [[ $SIGNAL -eq 1 ]]; then
    echo "Signal Received" >> "${BASEDIR}/${LOGFILE}"
    break

## Main Loop

LOOPCOUNT=0
CAM_FAILS=0
MAX_CAM_FAILS=5
while :; do

    # For debug loop termination
    if [[ -n $1 ]] && [[ $LOOPCOUNT -eq $1 ]]; then
	echo "Done" >> "${BASEDIR}/${LOGFILE}"
        break
    fi

    echo "Loop: $LOOPCOUNT" >> "${BASEDIR}/${LOGFILE}"

# 1. Check available memory storage

    FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
    echo "Remaining Free Memory: $FREEMEM MB" >> "${BASEDIR}/${LOGFILE}"

    # if remaining storage is < 200MB
    if [[ $FREEMEM -lt 200 ]]; then

        # 2 oldest data files
        read -r FILE1 FILE2 < <(ls -tr1 --file-type "${BASEDIR}/data/" | grep -v '/$' | head -n 2)

        # delete oldest files (-f to force)
        echo "Removing $FILE1, $FILE2" >> "${BASEDIR}/${LOGFILE}"
        rm -f "${BASEDIR}/data/${FILE1}" "${BASEDIR}/data/${FILE2}" 
    fi

# 2. Record video

    # run program --- terminate after 3 min 50 sec
    
    timeout -k 50s 3m rpicam-vid -n -t 210000 -o "raw.${VFILETYPE}" >> "${BASEDIR}/${LOGFILE}" 2>&1

    # check if camera interfacing failed
    rc=$?
    if [[ $rc -ne 0 ]]; then
        echo "Camera capture failed (exit $rc), failure #${CAM_FAILS}" >> "${BASEDIR}/${LOGFILE}"
        ((CAM_FAILS++))
        if (( CAM_FAILS >= MAX_CAM_FAILS )); then
            echo "Too many camera failures, exiting." >> "${BASEDIR}/${LOGFILE}"
            break
        fi
        continue
    fi

    if [[ ! -f "raw.${VFILETYPE}" ]]; then
        echo "Capture finished but raw file not found" >> "${BASEDIR}/${LOGFILE}"
        continue
    fi

# 3. Organize Data Output

    # define data file naming index
    FILEIDX=$((LOOPCOUNT + FILECOUNT + 1))

    # move and rename data files
    echo "Cataloging data files" >> "${BASEDIR}/${LOGFILE}"
    #mv data.txt "${BASEDIR}/data/data${FILEIDX}.txt"
    TIME_NOW=$(date +"%Y-%m-%d_%H-%M-%S")
    if ((ping_attempts < MAX_PING_ATTEMPTS)); then
        mv "raw.${VFILETYPE}" "${BASEDIR}/data/${TIME_NOW}_raw${FILEIDX}.${VFILETYPE}"
    else
        mv "raw.${VFILETYPE}" "${BASEDIR}/data/${TIME_NOW}X_raw${FILEIDX}.${VFILETYPE}"
    fi
    CAM_FAILS=0

    LOOPCOUNT=$((LOOPCOUNT + 1))

done
