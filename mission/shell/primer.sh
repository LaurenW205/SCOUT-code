#!/bin/bash

## Primes the RaspberryPi to run the missionDriver script on reboot
sudo systemctl daemon-reload
sudo systemctl enable mission.service
sudo systemctl start mission.service