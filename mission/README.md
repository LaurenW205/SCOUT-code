## The purpose of this code is handle SCOUT mission operations onboard the 2026 RockSat Launch

# How to run
To run this code, one must use the primer.sh script within the shell/ directory.

This script will prime the RaspberryPi to be ready for autonomous funciton after rebooting.

Simply type `./primer.sh` in the command line and reboot the RasPi to begin!

If you would like to run this script manually, use the command `./missionDriver.sh 1`

Remember to send Ctrl+C interrupts to start and exit video recording


# How to Update Code
To make changes to the code, edit the files in the src/ directory.

The code will recompile in the driver script so follow the 'How to run' instructions above.