#include "Telemetry.h"

void setup() {
   //this is where the unicorns and butterflies are born
   Serial.begin(9600);
}

void loop() {
   //magic goes here
   if (Serial.available() > 0) {
      char incomingBytes[30];
      int lengthIncoming = 30;
      lengthIncoming = Serial.readBytesUntil('\r', incomingBytes, 30);
      Address64 remAddress;
      Message newMessage;
      newMessage.setContent(incomingBytes, lengthIncoming);
      newMessage.sendTo(remAddress);
   }
}