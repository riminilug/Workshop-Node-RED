#include <Arduino.h>

const int LDR = A0;
const int BUTTON = 4;
const int RED = 15;
const int GREEN = 12;
const int BLUE = 13;

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"



/**************** Configuration from external file *******************/
#include "myconfig.h"



/************************ WiFi Access Point *********************************/
#ifndef MYCONFIG
#define WLAN_SSID       "myssid"
#define WLAN_PASS       "mypassword"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "mqtt.myserver"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""

#endif

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a topic for publishing photocell.
Adafruit_MQTT_Publish mqtt_photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/light");

// Setup a topic for publishing button.
Adafruit_MQTT_Publish mqtt_button = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/button");

// Setup a feed called 'rgb' for subscribing to changes.
Adafruit_MQTT_Subscribe mqtt_rgb = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/rgb");

void MQTT_connect();

void setup()
{
  pinMode(LDR, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  set_rgb_led("00ff00");
  delay(500);
  Serial.begin(115200);
  delay(10);


  Serial.println(F("Adafruit MQTT demo"));

  

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  //Serial.println(WLAN_PASS);

  set_rgb_led("ff0000");
 
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  set_rgb_led("0000ff");
  delay(500);
  set_rgb_led("00ff00");
  delay(500);
  set_rgb_led("0000ff");
  delay(500);
  set_rgb_led("00ff00");

  delay(1000);
  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqtt_rgb);




}


int photocell_threshold = 50;
int photocell_val_prev = -1;
int photocell_millis_prev = -1;
int button_val_prev = -1;


void loop()
{
  int photocell_val = analogRead(LDR);
  int button_val = digitalRead(BUTTON);

  //    Serial.print("LDR: ");
  //    Serial.println(photocell_val);
  //    Serial.print("BUTTON: ");
  //    Serial.println(button_val);


  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &mqtt_rgb) {
      char* rgb_val = (char*) mqtt_rgb.lastread;
      *(rgb_val+7)= 0x00;
      long rgb =  strtol(rgb_val+1, 0, 16);
      Serial.print(F(">>>>>>> Got: "));

      Serial.println(rgb_val);
      Serial.println(rgb);

      // Skip alpha
      int r = ((rgb>>16) );
      int g = ((rgb>>8) & 0xff);
      int b = ((rgb) & 0xff);

      Serial.println(r);
      Serial.println(g);
      Serial.println(b);


      analogWrite(RED, r*1023/255 );
      analogWrite(GREEN, g*1023/255 );
      analogWrite(BLUE, b*1023/255 );


    }
  }

  // Now we can publish stuff, if changed or every 10sec
  long diff = millis()-photocell_millis_prev;
  
  if ((abs(photocell_val - photocell_val_prev) > photocell_threshold) ||
      (diff > 10000))
  {
    Serial.print(F("\nSending photocell val "));
    Serial.print(photocell_val);
    Serial.print("...");
    

    if (! mqtt_photocell.publish(photocell_val)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
      photocell_val_prev = photocell_val;
      photocell_millis_prev = millis();
    }
  }

  if (button_val != button_val_prev) {
    if (! mqtt_button.publish(button_val)) {
      Serial.println(F("Failed"));
    } else {
      button_val_prev = button_val;
      Serial.println(F("OK!"));
    }
  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
  mqtt.disconnect();
}
*/
}


// rgb_val = '#ffffff'
void set_rgb_led(char* rgb_val){

  long rgb =  strtol(rgb_val, 0, 16);
 
  // Skip alpha
  int r = ((rgb>>16) );
  int g = ((rgb>>8) & 0xff);
  int b = ((rgb) & 0xff);

  
  
  analogWrite(RED, r*1023/255 );
  analogWrite(GREEN, g*1023/255 );
  analogWrite(BLUE, b*1023/255 );

}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");


  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    set_rgb_led("ff00ff"); 
    
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  set_rgb_led("00ff00");
}
