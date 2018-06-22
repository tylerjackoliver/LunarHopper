#include "Telemetry.h"
void setup() {
   Serial1.begin(115200, SERIAL_8N2);
   Serial.begin(9600);
}

void loop() {
   delay(10000);

   Message message;
   message.setStatusByte(0xFF);

   Recipient pc;
   pc.setAddresses(0x0013A200, 0x409FB676, 0x380D);

   pc.send(message);
}