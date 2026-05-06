## The purpose of this code is handle SCOUT mission operations onboard the 2026 RockSat Launch

# How to run
To set up this code when the service has been disabled, one must use the enable.sh script within the shell/ directory.

This script will prime the RaspberryPi to be ready for autonomous funciton after rebooting.

Simply type `./enable.sh` in the command line and reboot the RasPi to begin!

If you would like to run this script manually, use the command `./missionDriver.sh 1`

Changing the number input to the above command affects how many loops the bash script performs

# How to start recording
To begin recording, send a 2-3.3V input to the Raspi GPIO24 pin.

This is the signal the bash script is waiting for to begin recording.

# How to change video parameters
To change the length of the video recorded, go into the missionDriver.sh file and edit the run section.

In the line beginning with 'timeout' edit the timeout value to be 20 seconds + your desired video time (1m vid -> 20s 1m timeout).

Next change the time in the rpi-vid command to your desired recording time in milliseconds.

# How to Disable Code
To disable the boot up service, run the disable.sh script in the shell/ directory.