#!/bin/bash

### Drives mission code


## Boot Up & Initialization

BASEDIR="home/scout/Desktop/mission"

# create .log file
LOGFILE="mission.log"
echo -n "" > "${BASEDIR}/${LOGFILE}"
sleep 1
echo "Log file created at $(date)" >> "${BASEDIR}/${LOGFILE}"

sudo echo 536 > sys/class/gpio/export
sleep 1
sudo echo in > sys/class/gpio/gpio536/direction
echo "GPIO intialized" >> "${BASEDIR}/${LOGFILE}"

# check for residual data file
OLDFILE=$(ls -1 -- *.h264 2>"${BASEDIR}/${LOGFILE}" | head -n1 )

if [[ -n $OLDFILE ]]; then
    
    mv -- "$OLDFILE" "$dest"
    echo "Moved: $OLDFILE -> data/" >> "${BASEDIR}/${LOGFILE}"
else
    echo "No old .h264 files found." >> "${BASEDIR}/${LOGFILE}"
fi

MAINFILE="src/main.py"
VFILETYPE="h264"

# read true timestamp from file
TIMESTAMP=$(head -n 1 "timestamp.txt")
echo "Parent timestamp: ${TIMESTAMP}" >> "${BASEDIR}/${LOGFILE}"

# check memory allocation
FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
echo "Remaining Free Memory: $FREEMEM MB" >> "${BASEDIR}/${LOGFILE}"

# check current number of files to determine file index later on
FILECOUNT=$(find "${BASEDIR}/data" -maxdepth 1 -type f -name "*.${VFILETYPE}" 2>"${BASEDIR}/${LOGFILE}" | wc -l)

# Idle until LOW signal received on GPIO 24
while :; do
    SIGNAL=$(cat sys/class/gpio/gpio536/value)
    if [[ $SIGNAL -eq 1 ]]; then
    echo "Signal Received" >> "${BASEDIR}/${LOGFILE}"
    break
    fi
done

## Main Loop

LOOPCOUNT=0
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

    # run program --- terminate after 1 min 16 sec

    timeout -k 50s 3m rpicam-vid -n -t 210000 -o "${BASEDIR}/${TIMESTAMP}raw.${VFILETYPE}" >> "${BASEDIR}/${LOGFILE}" 2>&1


# 3. Organize Data Output

    # define data file naming index
    FILEIDX=$((LOOPCOUNT + FILECOUNT + 1))

    # move and rename data files
    echo "Cataloging data files" >> "${BASEDIR}/${LOGFILE}"
    #mv data.txt "${BASEDIR}/data/data${FILEIDX}.txt"
    mv "${BASEDIR}/${TIMESTAMP}raw.${VFILETYPE}" "${BASEDIR}/data/${TIMESTAMP}raw${FILEIDX}.${VFILETYPE}"

    LOOPCOUNT=$((LOOPCOUNT + 1))

done
