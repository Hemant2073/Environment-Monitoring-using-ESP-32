#include <TinyGPSPlus.h>
#include<Q2HX711.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define DHTPIN 19
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

//TODO: ESP32 MQTT user config
const char* ssid = "OPPO A15s"; // Wifi SSID
const char* password = "hemantkumar"; // Wifi Password
const char* username = "hemant207"; // my AskSensors username
const char* pubTopic = "publish/hemant207/Dc0ZSWntrzrE92gRvsAVUQQu9leP0UCw"; // publish/username/apiKeyIn
const char* pubTopic1 = "publish/hemant207/SxrMLATmXXljn2kO8To8uoct51NTre2F";
const unsigned int writeInterval = 2500; // write interval (in ms)
//AskSensors MQTT config
const char* mqtt_server = "mqtt.asksensors.com";
unsigned int mqtt_port = 1883;

// objects
WiFiClient askClient;
PubSubClient client(askClient);


TinyGPSPlus gps;

int  MQ2pin = 5;       //Gas Sensor
float sensorValue;      //Gas Sensor value

        
int ledpin = 18;

const byte MPS_OUT_pin = 2;
const byte MPS_SCK_pin = 4;
int avg_size = 10;
Q2HX711 MPS20N0040D(MPS_OUT_pin, MPS_SCK_pin);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
    Serial.println("*****************************************************");
    Serial.println("********** Program Start : ESP32 publishes NEO-6M GPS position to AskSensors over MQTT");
    Serial.print("********** connecting to WIFI : ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("->WiFi connected");
    Serial.println("->IP address: ");
    Serial.println(WiFi.localIP());

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    Serial.println("MQ2 is ready");
    pinMode(ledpin, OUTPUT);
    digitalWrite(ledpin,LOW);
    dht.begin();
    delay(2000);
}


void loop() {
  if (!client.connected())
  reconnect();
  client.loop();

  //updateSerial();GPS 
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  Serial.println("The sensrs readings are : ");
  displayInfo1();


  delay(200);
}

// GPS displayInfo
void displayInfo() {

if (gps.location.isValid()) {
double latitude = (gps.location.lat());
double longitude = (gps.location.lng());

Serial.println("********** Publish MQTT data to ASKSENSORS");
char mqtt_payload[50] = "";

snprintf (mqtt_payload, 50, "m1=%lf;%lf", latitude, longitude);

Serial.print("Publish message: ");
Serial.println(mqtt_payload);
client.publish(pubTopic, mqtt_payload);

Serial.println("> GPS data published");

Serial.println("*****************************************************");

delay(writeInterval);// delay 
} else {
Serial.println(F("INVALID"));
}

}

void displayInfo1() {


  //Gas sensor readings
  sensorValue = analogRead(MQ2pin);
  Serial.print("Sensor Value : ");
  Serial.println(sensorValue);
  delay(700);

  if(sensorValue>500){
      digitalWrite(ledpin,HIGH);
  }else{
    digitalWrite(ledpin,LOW);
  }

  //pressure sensor
  float avg_val = 0.0;
  for (int ii=0;ii<avg_size;ii++){
    avg_val += MPS20N0040D.read();
    delay(50);
  }
  avg_val /= avg_size;
  Serial.println(avg_val,0);
  delay(200);

float h = dht.readHumidity();
// Read temperature as Celsius (the default)
float t = dht.readTemperature();
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));

Serial.println("********** Publish MQTT data to ASKSENSORS");

char mqtt_payload1[50] = "";
char mqtt_payload2[50] = "";
char mqtt_payload3[50] = "";

snprintf (mqtt_payload1, 50, "m2=%d",avg_val );
snprintf (mqtt_payload2, 50, "m3=%lf", t);
snprintf (mqtt_payload3, 50, "m4=%lf", h);
Serial.print("Publish message: ");
Serial.println(mqtt_payload);

client.publish(pubTopic, mqtt_payload1);Serial.print(",");
client.publish(pubTopic, mqtt_payload2);Serial.print(",");
client.publishln(pubTopic, mqtt_payload3);
Serial.println("> Sensors data published");
Serial.println("********** End ");
Serial.println("*****************************************************");

delay(writeInterval);// delay 
} else {
Serial.println(F("INVALID"));
}

}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}


//MQTT reconnect
void reconnect() {
// Loop until we're reconnected
while (!client.connected()) {
Serial.print("********** Attempting MQTT connection...");
// Attempt to connect
if (client.connect("ESP32Client", username, "")) { 
Serial.println("-> MQTT client connected");
} else {
Serial.print("failed, rc=");
Serial.print(client.state());
Serial.println("-> try again in 5 seconds");
// Wait 5 seconds before retrying
delay(5000);
}
}
}

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");
for (int i = 0; i < length; i++) {
Serial.print((char)payload[i]);
}
Serial.println();
}

