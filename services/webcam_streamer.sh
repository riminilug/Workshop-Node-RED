#! /bin/sh

cd ~/src/mjpg-streamer/mjpg-streamer-experimental
mjpg_streamer -o "output_http.so -w ./www" -i"input_uvc.so -d /dev/video2"


