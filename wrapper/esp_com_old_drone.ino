#include <WiFi.h>
#ifndef IS_SERVER

#define LED 2
String HOST="192.168.4.1";

void setup(){

  Serial.begin(115200);
  pinMode(LED, OUTPUT);
    WiFi.begin("Pluto_2022_3514", "pluto5536");
   
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(100);
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);
  } 

  digitalWrite(LED,HIGH);
    
}

void loop(){
   
    
    WiFiClient client;
    IPAddress local_IP(192, 168, 4, 1);
    if (!client.connect(local_IP, 23)){
        Serial.write("NOT CLIENT CONNECT");
    }
 
    while(Serial.available()){
      uint8_t value = Serial.read();
      client.write(value);
    }
    client.flush();
    client.stop();
    
}

#endif