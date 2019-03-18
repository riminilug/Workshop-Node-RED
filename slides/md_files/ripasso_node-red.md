# Node-red
Ripasso, dubbi e approfondimenti
<img src="images/nodered.png" width="100%">

<--s-->
## I messaggi tra i nodi del flow

```
{
  "_msgid": "6789bd5c.987644",
  "topic": "temperatura",
  "payload": 18.5
}
```
<--s-->
## Il payload dei messaggi
<img src="images/payload.jpg" width="80%">

<--s-->

## payload più articolati (JSON)

```
{
  "_msgid": "69d566da.962a98",

  "topic": "stanza1",

  "payload": {
    "temperatura": 18,
    "umidita": 45,
    "luogo": "stanza nr1"
  }
}
```

<--s-->
## Altri attributi dei messaggi

```
{
  "_msgid": "e9f04e6f.160fb",

  "topic": "stanza1",

  "payload": 1487068882758,
  "temperatura": "18",
  "umidita": "45",
  "luogo": "stanza nr1"  
}
```


<--s-->
## Variabili globali e di flow
Lettura/scrittura tramite nodo

<img src="images/setglobalvar.jpg" width="50%">


<--s-->
## Variabili globali e di flow
Lettura/scrittura tramite funzione javascript

```
// Scrivo il valore
global.set("myglobalvar","value of variable");  

// ora il valore è accessibile agli altri nodi

// Leggo il valore
var var1 = global.get("myglobalvar");  

```


<--s-->
## Nodi utili #1
<img src="images/nr_inout.png" width="100%">

<--s-->
## Nodi utili #2
<img src="images/nr_functions.png" width="100%">

<--s-->
## Nodi utili #3
Dashboard: Creare interfacce utente sul browser
<img src="images/nr_dashboard.png" width="90%">
