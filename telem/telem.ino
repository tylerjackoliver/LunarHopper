#include "Telemetry.h"

// DIRECTIONS: comment/uncomment setup and loop for which Arduino you want to upload to

//********* Receiver; Remote/PC; Currently configured for Remote *********//
// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   Serial1.begin(115200, SERIAL_8N2);
//   Serial.println("Ready for transmission!");
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   if (Serial1.available() > 0) {
//     Serial.print("Coming from HOP(coord): ");
//     char incomingByte;
//     while(Serial1.available()>0){
//     incomingByte = Serial1.read();
//     Serial.write(incomingByte);
//     delay(10);
//     }
//     //echo back
//     Serial.print("\r\n");
//   }
// }

//********* Transmitter; The Hopper; XBee Coordinator *********//
void setup()
{
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