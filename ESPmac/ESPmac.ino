#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Esp Mac  ");
  Serial.println(WiFi.macAddress());
 
}

void loop() {
}
