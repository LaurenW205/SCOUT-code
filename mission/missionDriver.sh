#!/bin/bash

### Drives mission code


## Boot Up & Initialization

BASEDIR="/home/scout/Desktop/mission"

# create .log file
LOGFILE="mission.log"
echo -n "" > "${BASEDIR}/${LOGFILE}"
echo "Log file created at $(date)" >> "${BASEDIR}/${LOGFILE}"

# check memory allocation
FREEMEM=$(df -m / | awk 'NR==2 {print $4}')
echo "Remaining Free Memory: $FREEMEM MB" >> "${BASEDIR}/${LOGFILE}"

# check current number of files to determine file index later on
FILECOUNT=$(find "${BASEDIR}/data" -maxdepth 1 -type f -name '*.txt' 2>"${BASEDIR}/${LOGFILE}" | wc -l)

## Main Loop
trap 'exit 0' INT

LOOPCOUNT=0
while :; do

    # For debug loop termination
    if [[ -n $1 ]] && [[ $LOOPCOUNT -eq $1 ]]; then
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

# 2. Call/Run files

    # compile and run code from /src --- terminate after 1 min 10 sec

    #timeout -k 10s 1m ./"${BASEDIR}${MAINFILE}" >> "${BASEDIR}${LOGFILE}" 2>&1 # old implementation for .exe
    timeout -k 10s 1m python "${BASEDIR}/src/main.py" >> "${BASEDIR}/${LOGFILE}" 2>&1

        # if error, must recompile from /src


# 3. Organize Data Output

    # define data file naming index
    FILEIDX=$((LOOPCOUNT + FILECOUNT + 1))

    # move and rename data files
    echo "Cataloging data files" >> ${BASEDIR}/${LOGFILE}
    mv "${BASEDIR}/data.txt" "${BASEDIR}/data/data${FILEIDX}.txt"
    mv "${BASEDIR}/raw.mp4" "${BASEDIR}/data/raw${FILEIDX}.mp4"

    LOOPCOUNT=$((LOOPCOUNT + 1))

done