// Aquariumcomputer "Aquacom v0.1"
 // Copyright (c) 2021 by Marcus Lorenz
 // -----------------------------------
 // Temperaturmessung per DS18B20
 
#include <SPI.h>     
#include "Ethernet.h"
#include "PubSubClient.h"
#include <OneWire.h>                                                    // OneWire
#include <DallasTemperature.h>                                          // Dallas Temperatursensor

#define ONE_WIRE_BUS 2                                                  // Pin 2 DS18B20

#define CLIENT_ID       "AquacomUNO"
//#define inTopic         "aquacomUNO/cmd"
//#define outTopic        "aquacomUNO/state"

#define PUBLISH_DELAY   15000

#define ledPin 13
#define relay1Pin 8
#define relay2Pin 7

String ip = "";
char msgTempC[100];
char msgRelay1[100];
char msgRelay2[100];
char msgRelay3[100];
char msgRelay4[100];


char LWLTopic[] = "aquacomUNO/LWL";                                     // LWLTopic
char inTopic[] = "";                                      // InTopic
char outTopic[] = "aquacomUNO/state";                                   // OutTopic
 

bool relay1state = LOW;
bool pir = LOW;
bool startsend = HIGH;
int lichtstatus;
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06};

EthernetClient ethClient;
PubSubClient mqttClient;
OneWire oneWire(ONE_WIRE_BUS);                                          // Temperatursensor einbinden
DallasTemperature sensors(&oneWire);

long previousMillis;

void setup() {
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  digitalWrite(LED_BUILTIN,HIGH);

  // setup serial communication

  Serial.begin(9600);
  while (!Serial) {};
  Serial.println(F("MQTT Arduino Demo"));
  Serial.println();

  // setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {
    //Serial.println(F("Unable to configure Ethernet using DHCP"));
    for (;;);
  }

  Serial.println(F("Ethernet configured via DHCP"));
  Serial.print("IP address: ");
  Serial.println(Ethernet.localIP());
  Serial.println();

  ip = String (Ethernet.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[3]);
  //Serial.println(ip);

  // setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer( "192.168.1.94", 1883);
  //Serial.println(F("MQTT client configured"));
  mqttClient.setCallback(callback);

  sensors.begin();                                                      // Sensor initialisieren
  Serial.println(F("DS18B20 Sensor initialisiert"));

  Serial.println();
  Serial.println(F("Fertig zum Daten senden"));
  
  previousMillis = millis();
  
  mqttClient.publish("aquacomUNO/state/ip", ip.c_str());
}

void loop() {

  // it's time to send new data?
  if (millis() - previousMillis > PUBLISH_DELAY) {
    sendData();
    previousMillis = millis();
  }

  mqttClient.loop();
}

void sendData() {
  char msgBuffer[20];
  static char floatStr[15];
  
  String payLoadTempC;
  String payLoadRelay1;
  String payLoadRelay2;
  String payLoadRelay3;
  String payLoadRelay4;
  
  String Relay1;
  String Relay2;
  String Relay3;
  String Relay4;
  String tempC;
  
  float  tempCWertFloat;
  

  sensors.requestTemperatures();                                         // Temperatur messen

  tempCWertFloat = sensors.getTempCByIndex(0);                           // Messwert holen
  dtostrf( tempCWertFloat,7, 2, floatStr);                               // Zahlenwert in String wandeln
  tempC = floatStr;
  tempC.trim();
  payLoadTempC = "{\"Wassertemperatur\":\"";                             // Nachricht zusammenbauen
  payLoadTempC = payLoadTempC + tempC + "\"}";                           // Messwert dazufügen
  payLoadTempC.toCharArray(msgTempC, 100 );                              // für publish-Funktion umwandeln

  Relay1 = digitalRead(relay1Pin);                                       // Relay1 Status abfragen
  payLoadRelay1 = "{\"Relay1\":\"";                                      // Nachricht zusammenbauen
  payLoadRelay1 = payLoadRelay1 + Relay1 + "\"}";                           // Messwert dazufügen
  payLoadRelay1.toCharArray(msgRelay1, 100 );                              // für publish-Funktion umwandeln
  
  Relay2 = digitalRead(relay2Pin);                                       // Relay1 Status abfragen
  payLoadRelay2 = "{\"Relay2\":\"";                                      // Nachricht zusammenbauen
  payLoadRelay2 = payLoadRelay2 + Relay2 + "\"}";                           // Messwert dazufügen
  payLoadRelay2.toCharArray(msgRelay2, 100 );                              // für publish-Funktion umwandeln


  Serial.println( msgTempC );
  Serial.print("Temperatur: ");
  Serial.print(tempC);
  Serial.println("°C");

  Serial.println( msgRelay1 );  
  Serial.println( msgRelay2 );
  
  if (mqttClient.connect(CLIENT_ID)) {
    mqttClient.publish( outTopic, msgTempC );                                              // MQTT-Nachricht an Brocker senden
    mqttClient.publish( outTopic, msgRelay1 );
    mqttClient.publish( outTopic, msgRelay2 );
    mqttClient.publish("aquacomUNO/state/ip", ip.c_str());
    mqttClient.subscribe("aquacomUNO/cmd/relay1");
    mqttClient.subscribe("aquacomUNO/cmd/relay2");
    mqttClient.subscribe("aquacomUNO/cmd/relay3");
    mqttClient.subscribe("aquacomUNO/cmd/relay4");
    
    if (startsend) {
      // mqttClient.publish("home/br/nb/relay", (relaystate == LOW) ? "OPEN" : "CLOSED");
      mqttClient.publish("aquacomUNO/state/ip", ip.c_str());
      startsend = LOW;
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");//MQTT_BROKER

  int i = 0;
  char message_buff[100];

  for ( i=0;i<length;i++)
  {message_buff[i] = payload[i]; }
message_buff[i] = '\0';
String msgString = String(message_buff);
Serial.println(topic);
Serial.println("Payload: " + msgString);

if((strcmp(topic, "aquacomUNO/cmd/relay1") == 0)&&(strcmp(message_buff, "true") == 0)) {
  Serial.println("Relais1 eingeschaltet");
  digitalWrite(relay1Pin, LOW);
  }

if((strcmp(topic, "aquacomUNO/cmd/relay1") == 0)&&(strcmp(message_buff, "false") == 0)) {
  Serial.println("Relais1 ausgeschaltet");
  digitalWrite(relay1Pin, HIGH);
  }

if((strcmp(topic, "aquacomUNO/cmd/relay2") == 0)&&(strcmp(message_buff, "true") == 0)) {
  Serial.println("Relais2 eingeschaltet");
  digitalWrite(relay2Pin, LOW);
  }

if((strcmp(topic, "aquacomUNO/cmd/relay2") == 0)&&(strcmp(message_buff, "false") == 0)) {
  Serial.println("Relais2 ausgeschaltet");
  digitalWrite(relay2Pin, HIGH);
  }
}
