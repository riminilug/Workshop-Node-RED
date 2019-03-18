<img src="images/mqttorg.png" width="60%">

a machine-to-machine (M2M)/"Internet of Things" connectivity protocol

<br><br>
[http://mqtt.org/]

<--s-->
## I 3 attori di un sistema MQTT
<img src="images/mqttdiagram.png" width="80%">

- TCP/IP  port 1883 <!-- .element: class="fragment" -->
- TCP/IP  port 8883 (SSL) <!-- .element: class="fragment" -->

<--s-->
## I topics MQTT
<img src="images/mqttdiagramtopics.png" width="100%">

<--s-->
## Quality of Service
I tre livelli di QoS definiscono l'affidabilità nel recapito del messaggio mqtt.

<img src="images/mqtt_qos.png" width="100%">


<--s-->
## Dove installo il broker ?

- Sul Raspberry PI (assieme a Node-Red) <!-- .element: class="fragment" -->
- Su un server nella nostra rete (LAN) <!-- .element: class="fragment" -->
- Su un server Internet (VPS o servizi cloud) <!-- .element: class="fragment" -->

<--s-->
<img src="images/mosquitto.png" width="20%">

Un broker MQTT (v3.1) Open Source e multipiattaforma

https://mosquitto.org/


<--s-->
## Mosquitto sul Raspberry PI
```
wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key
sudo apt-key add mosquitto-repo.gpg.key

cd /etc/apt/sources.list.d/

sudo wget http://repo.mosquitto.org/debian/mosquitto-jessie.list

sudo apt-get update

sudo apt-get install mosquitto



** Il file di configurazione: **
    -->   /etc/mosquitto/mosquitto.conf
```

<--s-->
## Node-Red e MQTT

Node-Red contiene i nodi di input e output per la connessione ad un broker MQTT.

<img src="images/mqtt-nodered.png" width="40%">
