#!/bin/bash

## Diables reboot funcitonality for debugging
sudo systemctl daemon-reload
sudo systemctl stop mission.service
sudo systemctl disable mission.service
