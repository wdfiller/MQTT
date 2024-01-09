// Arduino Nano RP240 Connect
// 1/8/2024

#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"

const char* mqtt_server = "192.168.0.89";
const int mqtt_port = 1883;
const char* mqtt_username = "david";
const char* mqtt_password = "MadeNUSA1968";
const char* topic = "Temperature";
const char* id = "123456";
int mqttConnectAttempts = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;

int analogPin=A0;
float volts;
float tempC; 
float tempF;
const int arraySize=500;
int countsArray[arraySize];

void(* resetFunc) (void) = 0;
void setup()
{
  //Serial.begin(19200);
  //while (!Serial);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  if (!IMU.begin())
  {
    while (1);
  }

  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
}

void loop(){

  // hold here until WIFI is established
  while (status != WL_CONNECTED){
    connectWIFI();
  }

  //hold here until the MQTT broker is connected
  while (!client.connected()) {
    connectMQTTBroker();
  }

  digitalWrite(LEDB, HIGH);

  //read the voltage on the pin and store it in an array
  for(int i=0; i < arraySize; i++){
    countsArray[i] = analogRead(analogPin);
    delay(1);
  }

  volts = 3.27 * getAverageCounts() / (1020-3);
  tempC = 100.0 * (volts-0.5);
  tempF = 9.0 * tempC / 5.0 + 32.0;

  // Publish temperature every 10 seconds
  if(client.connected()){
    client.publish(topic, String(tempF,1).c_str());
  }

  digitalWrite(LEDB, LOW);
  delayAndBlinkGreen(10);
}

// get average counts
double getAverageCounts(){
  double sum = 0;
  for(int i=0; i < arraySize; i++){
    sum+=countsArray[i];
  }
  return sum/arraySize;
}

// connect to WIFI
void connectWIFI(){
  // check if a WIFI module exists
  if (WiFi.status() == WL_NO_MODULE) {
    delayAndBlinkRBG(30);
    resetFunc();
  }

  // check for latest firmware
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    delayAndBlinkRed(2);
  }

  status = WiFi.begin(ssid, pass);
  delayAndBlinkBlue(10);
}

//Connect to MQTT Broker
void connectMQTTBroker() {
  mqttConnectAttempts+=1;
  client.setServer(mqtt_server, mqtt_port);
  if (client.connect(id, mqtt_username, mqtt_password)) {
  } else {
    delayAndBlinkRedBlue(5);
  }
  if(mqttConnectAttempts == 5){
    resetFunc();
  }
}

//delay and blink built in LED
void delayAndBlinkRed(int seconds) {
  for (int index = 0; index < seconds; index++) {
    digitalWrite(LEDR, HIGH);
    delay(500);
    digitalWrite(LEDR, LOW);
    delay(500);
  }
}
//delay and blink built in LED
void delayAndBlinkBlue(int seconds) {
  for (int index = 0; index < seconds; index++) {
    digitalWrite(LEDB, HIGH);
    delay(500);
    digitalWrite(LEDB, LOW);
    delay(500);
  }
}
//delay and blink built in LED
void delayAndBlinkGreen(int seconds) {
  for (int index = 0; index < seconds; index++) {
    digitalWrite(LEDG, HIGH);
    delay(500);
    digitalWrite(LEDG, LOW);
    delay(500);
  }
}

//delay and blink built in LED
void delayAndBlinkRedBlue(int seconds) {
  for (int index = 0; index < seconds; index++) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    delay(500);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, HIGH);
    delay(500);
  }
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
}

//delay and blink built in LED
void delayAndBlinkRBG(int seconds) {
  for (int index = 0; index < seconds; index++) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, LOW);
    delay(500);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, LOW);
    delay(500);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);
    delay(500);
  }
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, LOW);
}