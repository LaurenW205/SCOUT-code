#!/bin/bash

### Drives mission code


## Boot Up & Initialization

cd home/scout

BASEDIR="Desktop/mission"

# create .log file
LOGFILE="mission.log"
echo -n "" > "${BASEDIR}/${LOGFILE}"
echo "Log file created at $(date)" >> "${BASEDIR}/${LOGFILE}"

MAINFILE="src/main.py"
VFILETYPE="h264"

# check memory allocation
FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
echo "Remaining Free Memory: $FREEMEM MB" >> "${BASEDIR}/${LOGFILE}"

# check current number of files to determine file index later on
FILECOUNT=$(find "${BASEDIR}/data" -maxdepth 1 -type f -name "*.${VFILETYPE}" 2>"${BASEDIR}/${LOGFILE}" | wc -l)

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

    # if remaining storage is < 100MB
    if [[ $FREEMEM -lt 100 ]]; then

        # 2 oldest data files
        read -r FILE1 FILE2 < <(ls -tr1 --file-type "${BASEDIR}/data/" | grep -v '/$' | head -n 2)

        # delete oldest files (-f to force)
        echo "Removing $FILE1, $FILE2" >> "${BASEDIR}/${LOGFILE}"
        rm -f "${BASEDIR}/data/${FILE1}" "${BASEDIR}/data/${FILE2}" 
    fi

# 2. Record video

    # run program --- terminate after 1 min 16 sec

    
    timeout -k 16s 1m rpicam-vid -n -t 60000 -o "raw.${VFILETYPE}" >> "${BASEDIR}/${LOGFILE}" 2>&1


# 3. Organize Data Output

    # define data file naming index
    FILEIDX=$((LOOPCOUNT + FILECOUNT + 1))

    # move and rename data files
    echo "Cataloging data files" >> ${BASEDIR}/${LOGFILE}
    #mv data.txt "${BASEDIR}/data/data${FILEIDX}.txt"
    mv "raw.${VFILETYPE}" "${BASEDIR}/data/raw${FILEIDX}.${VFILETYPE}"

    LOOPCOUNT=$((LOOPCOUNT + 1))

done