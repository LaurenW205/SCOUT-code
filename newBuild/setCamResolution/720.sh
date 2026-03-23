#!/bin/bash

v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1280,height=720
