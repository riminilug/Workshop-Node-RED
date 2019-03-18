#! /bin/sh

#----------------------------------------------------
# Create multiple instances of docker node-red
#----------------------------------------------------
# Used to create and manage node-red  instances 
# for a Node-RED workshop
#
# Author: Ivan Tarozzi (2019)
#         itarozzi@gmail.com
#----------------------------------------------------


usage() { echo "Usage: $0 start|stop|install_modules|sync  [-i <nr_of_instances>] " 1>&2; exit 1; }


NODERED_IMAGE=nodered/node-red-docker
NR_INSTANCES=2
NR_PREFIX=nodered_ws

#------------------------------------------
# Start node-red
#------------------------------------------
start() {
  echo "creating $NR_INSTANCES instances"
  for i in `seq 1 $NR_INSTANCES`;
  do
    echo 'Creating node-red instance #'$i
    PORT=`expr 1890 + $i`
    echo 'Port: '$PORT
    
    DATA_DIR=~/node-red-data/nodered_ws$i
    echo 'DATA_DIR: '$DATA_DIR

    mkdir -p $DATA_DIR
    chmod 777 $DATA_DIR

    docker run --rm -d -p $PORT:1880 -v $DATA_DIR:/data --name $NR_PREFIX$i $NODERED_IMAGE
  done
}


#------------------------------------------
# Stop nodered
#------------------------------------------
stop() {
  for i in `docker ps|grep $NR_PREFIX|cut -d' ' -f1`;
  do
    echo 'Stopping node-red instance #'$i
    docker stop $i
  done
}


#------------------------------------------
# Install node-red modules using node-red-admin
#------------------------------------------
install_modules() {

  # Work on instance 1
  set -o xtrace
  node-red-admin target http://localhost:1891
  node-red-admin install node-red-contrib-chatbot
  node-red-admin install node-red-node-arduino
  node-red-admin install node-red-contrib-mjpgcamera
  node-red-admin install node-red-contrib-moment
  node-red-admin install node-red-contrib-date
  node-red-admin install node-red-contrib-mqttdb
  node-red-admin install node-red-contrib-msg-speed
  node-red-admin install node-red-contrib-smartswitch
  node-red-admin install node-red-dashboard
  node-red-admin install node-red-node-sqlite
  node-red-admin install node-red-node-mongodb
  node-red-admin install node-red-contrib-fan
  node-red-admin install node-red-contrib-traffic
  node-red-admin install node-red-contrib-uibuilder

  set +o xtrace
}

#----------------------------------------------
# Sync node-red configuration from #1 to others
#----------------------------------------------
sync_instances()
{

  if [ $NR_INSTANCES -le 2 ];   then
    echo "ERROR: Select at least 2 instances"
    exit
  fi

  DATA_DIR_SRC=~/node-red-data/nodered_ws1
  for i in `seq 2 $NR_INSTANCES`;
  do
    DATA_DIR=~/node-red-data/nodered_ws$i

    echo 'Sync node-red instance #1 to #'$i' - '$DATA_DIR
        
    mkdir -p $DATA_DIR
    rsync -avr -q --exclude=flows.json  $DATA_DIR_SRC/ $DATA_DIR 
    
    
  done

  echo "Sync completed !"

}


#------------------------------------------
# Parse script arguments
#------------------------------------------
[ $# -eq 0 ] && usage

subcommand=$1
shift

while getopts ':i:' arg; do

  case $arg in
    i) # Specify p value.
      NR_INSTANCES=${OPTARG}
      ;;
   h | *) # Display help.
      usage
      exit 0
      ;;
  esac
done

case $subcommand in
  start) #start node-red instances
    echo "Start node-red instance"
    start
    ;;
  stop) # kill all node-red-instances
    stop 
    ;;
  install_modules)  # install additional modules to node-red instance #1
    install_modules
    ;;
  sync)
    sync_instances
    ;;
  
  *)    # option not allowed
    usage;;
esac

