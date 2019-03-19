![RiminiLUG Logo](/slides/images/logo-riminilug.png)


Workshop: IoT con Node-RED 
===========================

Questo repository contiene il materiale del workshop tenuto il 19 marzo 2019


Tutti i software utilizzati nel workshop sono **Liberi e Open Source**, così come tutte le risorse ed i sorgenti contenuti in questo repository.


Contenuto del presente repository
---------------------------------
Sono presenti una serie di directory il cui contenuto è di seguito descritto.

### slides

Contiene il materiale a supporto del workshop (slides) realizzato in formato HTML5 e Markdown, utilizzando il software revealjs.


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


