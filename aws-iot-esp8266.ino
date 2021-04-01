/* ESP8266 AWS IoT
 *  
 * Author: David Piracoca
 */

#include "DHT.h"
#include "secrets.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"
extern "C" {
#include "libb64/cdecode.h"
}

#define DHTTYPE DHT11
#define dht_dpin D4
#define AWS_IOT_GET_TOPIC   "$aws/things/MyESP32/shadow/get"
#define AWS_IOT_UPDATE_TOPIC "$aws/things/MyESP32/shadow/update"

WiFiClientSecure net;
void messageHandler(char* topic, byte* payload, unsigned int len);
PubSubClient client(AWS_IOT_ENDPOINT, 8883, messageHandler, net); 

//sensor
DHT dht(dht_dpin, DHTTYPE);

void wifiConnectMqtt() {
  Serial.print("Connecting to "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult();
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // get current time, otherwise certificates are flagged as expired  
  setCurrentTime();

  uint8_t binaryCA[AWS_CERT_CA.length() * 3 / 4];
  int len = b64decode(AWS_CERT_CA, binaryCA);
  net.setCACert(binaryCA, len);

  uint8_t binaryCert[AWS_CERT_CRT.length() * 3 / 4];
  len = b64decode(AWS_CERT_CRT, binaryCert);
  net.setCertificate(binaryCert, len);
  
  uint8_t binaryPrivate[AWS_CERT_PRIVATE.length() * 3 / 4];
  len = b64decode(AWS_CERT_PRIVATE, binaryPrivate);
  net.setPrivateKey(binaryPrivate, len);
}

void connectAWS() {
    Serial.print("PubSubClient connecting to: "); Serial.print(AWS_IOT_ENDPOINT);
    while ( ! client.connected()) {
      Serial.print(".");
      client.connect(THINGNAME);
    }
    Serial.println(" connected");
    client.subscribe(AWS_IOT_UPDATE_TOPIC);
}

float temperature;
float humidity;

void publishMessage() {
  StaticJsonDocument<200> doc;

  float tempTemperature = dht.readTemperature();
  float tempHumidity = dht.readHumidity();

  if(tempTemperature > 0 && tempHumidity > 0){
    if(temperature != tempTemperature || humidity != tempHumidity){
      temperature = tempTemperature;
      humidity = tempHumidity;
    
      doc["time"] = millis();
      doc["temperature"] = temperature;
      doc["Current humidity"] = humidity;
      char jsonBuffer[512];
      serializeJson(doc,jsonBuffer);
      client.publish(AWS_IOT_GET_TOPIC, jsonBuffer);
      Serial.print("Published: "); Serial.println(jsonBuffer);
     }
  }
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  const bool encender = doc["encender"];

  if (encender){
    Serial.println("encendido");
    digitalWrite(D8, HIGH);
  }else{
    Serial.println("apagado");
    digitalWrite(D8, LOW); 
  }
  
  Serial.println(message);
}

int b64decode(String b64Text, uint8_t* output) {
  base64_decodestate s;
  base64_init_decodestate(&s);
  int cnt = base64_decode_block(b64Text.c_str(), b64Text.length(), (char*)output, &s);
  return cnt;
}

void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: "); Serial.print(asctime(&timeinfo));
}

void setup() {
  Serial.begin(115200); Serial.println();
  Serial.println("ESP8266 AWS IoT Example");
  wifiConnectMqtt();
  pinMode(D8, OUTPUT);
}

void loop() {
  if ( ! client.connected()) {
    connectAWS();
  }
  client.loop();
  publishMessage();
  delay(100);
}
