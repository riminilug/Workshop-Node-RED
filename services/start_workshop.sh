#! /bin/sh

#===============================================
# Avvia tutti i servizi utilizzati nel workshop
#===============================================

#1. Avvia il broker mqtt mosquitto
docker start mosquitto &

#2. Avvia le istanze node-red (10)
#./node-red-multi.sh start -i 10

#3. Avvia lo streaming della webcam
./webcam_streamer.sh &

#4. Avvia il webserver locale per le risorse e le slides
docker start nginx-static &


