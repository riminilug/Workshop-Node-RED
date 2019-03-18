<img src="images/telegram.jpg" width="40%">

E' possibile sfruttare Telegram e il suo servizio di Bot per integrare applicazioni e servizi IoT

<br><br>
[ https://telegram.org/ ]

<--s-->
## Telegram Bot
Sono account speciali, legati ad applicazioni, con cui gli utenti Telegram possono interagire attraverso:

- Invio di messaggi  <!-- .element: class="fragment" -->
- Invio di Comandi <!-- .element: class="fragment" -->
- Custom Keyboard (pulsanti) <!-- .element: class="fragment" -->

<img src="images/bot_keyboard1.jpg" width="15%"> <!-- .element: class="fragment" -->
<img src="images/bot_keyboard2.jpg" width="15%"> <!-- .element: class="fragment" -->

<--s-->
## Perchè usare i Telegram Bot ?

- posso interagire con il mio sistema attraverso internet senza esporlo direttamente <!-- .element: class="fragment" -->
- utilizzo la sicurezza intrinseca di Telegram <!-- .element: class="fragment" -->
- posso sfruttare il sistema di notifiche di Telegram <!-- .element: class="fragment" -->


<--s-->
## Come creo un Bot ?
<img src="images/botfather.png" width="40%">

https://core.telegram.org/bots#6-botfather

<--s-->
## Come creo un Bot ?

```
@botfather

  /newbot

  /setname

  /setcommands


  --->  Token per API HTTP
```

<--s-->
## Come utilizzo un Bot con node-red ?

- mi serve il Token di un Bot
- mi servono dei nodi aggiuntivi

<br>
**node-red-contrib-chatbot**<!-- .element: class="fragment" -->

https://flows.nodered.org/node/node-red-contrib-chatbot <!-- .element: class="fragment" -->
<--s-->
<img src="images/chatbot.jpg" width="100%">
