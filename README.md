![FLR Logo](/slides/images/logo-flr.png)

FabLab Romagna @ SMD2018
========================

Workshop: IoT con Node-RED e RaspberryPI
========================================

Questo repository contiene il materiale del workshop presso lo School Maker Day 2018 - Opificio Golinelli - Bologna.


Il workshop, rivolto principalmente a docenti della scuola secondaria, descrive l'utilizzo del sw **[Node-RED](https://nodered.org)** per realizzare un sistema IoT.

Tutti i software utilizzati nel workshop sono **Liberi e Open Source**, così come tutte le risorse ed i sorgenti contenuti in questo repository.

Architettura del sistema
------------------------

Durante il workshop, erogato in modalità BYOD, verranno utilizzate istanze di Node-RED su PC (erogate attraverso **docker**) e istanze eseguite su alcuni **RaspberryPI** messi a disposizone dall'associazione FabLab Romagna APS.

Inoltre, per realizzare una rete IoT, vengono utilizzati una serie di moduli ESP8266, collegati in WIFI e protocollo **MQTT** a PC e RaspberryPI, realizzando così una rete di sensori remoti. 
I moduli ESP8266 utilizzati sono i **Witty**.

Una serie di sensori e attuatori sono inoltre collegati direttamente ai RaspberyPI utilizzando il sistema GrovePI.


Contenuto del presente repository
---------------------------------
Sono presenti una serie di directory il cui contenuto è di seguito descritto.

### slides

Contiene il materiale a supporto del workshop (slides) realizzato in formato HTML5 e Markdown, utilizzando il software revealjs.

Inoltre, nella sottodirectory node-red-json, contiene una serie di esempi Node-RED utilizzati durante il workshop.


### esp8266

Contiene il codice sorgente per programmare i moduli Witty ESP8266 e interfacciarli via WIFI e MQTT al sistema. La programmazione dei moduli Witty avviene attraverso IDE Arduino opportunamente preparato.


### arduino

Contiene il codice sorgente per arduino, per dimostrare come sia possibile controllare, attraverso Node-RED, un Arduino collegato al PC tramite USB. La comunicazione Node-RED / Arduino avviene tramite protocollo firmata.
Inoltre una elaborazione locale di un programma permette la gestione degli I/O dell'Arduino.
In questo caso un anello di 16 NeoPixel viene comandato secondo diversi scenari e valori inviati da PC.


### services

Contiene una serie di script utili ad avviare servizi utilizzati durante il workshop

* `start_workshop.sh` avvia tutti i servizi utilizzati nel workshop
* `webcam_streamer.sh` avvia uno streming HTTP di una webcam connessa al PC. Utilizza il sw **mjpg-streamer** che quindi deve essere installato e configurato opportunamente
* `node-red-multi.sh` avvio di n-istanze Node-RED su un singolo PC. Inoltre lo script permette di configurare i nodi aggiuntivi nelle singole istanze.

In questo caso le varie istanza sono realizzate attraverso una serie di contenitori **[docker](https://www.docker.com/)** e sono raggiungibi dalla porta 1891 in su.


