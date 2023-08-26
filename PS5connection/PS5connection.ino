#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>



void setup() {
  Serial.begin(115200);

  ps5.begin("D0:BC:C1:98:2E:F3"); //replace with MAC address of your controller
  Serial.println("Ready.");
}

void loop() {
  if (ps5.isConnected()) {
    Serial.println("Connected!");
  }
  else{
    Serial.println("NOT Connected!");
    }
  delay(3000);
}
