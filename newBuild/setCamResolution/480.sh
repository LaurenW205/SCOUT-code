#!/bin/bash

v4l2-ctl --device=/dev/video0 --set-fmt-video=width=640,height=480
