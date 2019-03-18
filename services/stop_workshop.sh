#! /bin/sh

#===============================================
# Avvia tutti i servizi utilizzati nel workshop
#===============================================

#1. Arresta il broker mqtt mosquitto
docker stop mosquitto

#2. Arresta le istanze node-red (10)
./node-red-multi.sh stop -i 10

#3. Arresta lo streaming della webcam
killall mjpg_streamer

#4. Arresta il webserver locale per le risorse e le slides
docker stop nginx-static


