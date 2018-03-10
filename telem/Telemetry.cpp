//
//  Telemetry.cpp
//  Lunar Hopper
//
//  Created by Shane Lawson on 10/03/2018.
//

#include <Arduino.h>
#include "Telemetry.h"
#define START_DELIMETER 0x7E

Address64::Address64()
{
   //for now just create with REM address
   addressBytes[0] = 0x00;
   addressBytes[1] = 0x13;
   addressBytes[2] = 0xA2;
   addressBytes[3] = 0x00;
   addressBytes[4] = 0x40;
   addressBytes[5] = 0xA1;
   addressBytes[6] = 0xA0;
   addressBytes[7] = 0x43;
}

const char *Address64::getAddressPtr() const
{
   return &addressBytes[0];
}

Message::Message() 
{
   Serial.begin(9600);
   Serial.print("Opening channel for message...");
   Serial1.begin(115200, SERIAL_8N2);
   Serial.println("Done!");

   bytesTransmitted = 0;
   contentLength = MAX_CONTENT_BYTES;

   frameIDBytes[0] = 0x10;
   frameIDBytes[1] = 0x00; //turn off response from receiver. Use this later for confirmation of packet sent, but ease of bidirectional comms, handling a single case is easier.
   address16[0] = 0xFF;
   address16[1] = 0xFE;
   other[0] = 0x00;
   other[1] = 0x00;

   checksum = 0x00;
}

void Message::setContent(const char *contentIn, int length)
{
   if (length <= MAX_CONTENT_BYTES) {
      for (int i = 0; i < length; i++) {
         content[i] = contentIn[i];
      }
      contentLength = length;
   }
}

void Message::sendTo(const Address64 address) 
{
   Serial.print("Sending message...");
   this->calcChecksumAndLength(address);

   this->sendByte(START_DELIMETER);
   this->sendByte(lengthBytes[0]);
   this->sendByte(lengthBytes[1]);
   this->sendByte(frameIDBytes[0]);
   this->sendByte(frameIDBytes[1]);
   const char *addressPtr = address.getAddressPtr();
   for (int i = 0; i < 8; i++) {
      this->sendByte(addressPtr[i]);
   }
   this->sendByte(address16[0]);
   this->sendByte(address16[1]);
   this->sendByte(other[0]);
   this->sendByte(other[1]);
   for (int i = 0; i < contentLength; i++) {
      this->sendByte(content[i]);
   }
   this->sendByte(checksum);
   Serial.println("Done!");
}

void Message::calcChecksumAndLength(const Address64 address)
{
   unsigned char sum = 0;
   unsigned char bytes = 0;

   sum += frameIDBytes[0];
   bytes++;
   sum += frameIDBytes[1];
   bytes++;

   const char* addressPtr = address.getAddressPtr();
   for (int i = 0; i < 8; i++) {
      sum += addressPtr[i];
      bytes++;
   }

   sum += address16[0];
   bytes++;
   sum += address16[1];
   bytes++;
   sum += other[0];
   bytes++;
   sum += other[1];
   bytes++;

   for (int i = 0; i < contentLength; i++) {
      sum += content[i];
      bytes++;
   }

   checksum = 0xFF - sum;
   lengthBytes[0] = 0x00;
   lengthBytes[1] = bytes;
}

void Message::sendByte(char input)
{
   if (bytesTransmitted <= MAX_BYTES) 
   {
      switch (input) {
         case 0x7E:
            if (bytesTransmitted != 0) 
            {
               Serial1.write(0x7D);
               bytesTransmitted++;
               Serial1.write(0x5E);
            }else{
               Serial1.write(input);
            }
            break;

         case 0x7D:
            Serial1.write(0x7D);
            bytesTransmitted++;
            Serial1.write(0x5D);
            break;

         case 0x11:
            Serial1.write(0x7D);
            bytesTransmitted++;
            Serial1.write(0x31);
            break;

         case 0x13:
            Serial1.write(0x7D);
            bytesTransmitted++;
            Serial1.write(0x33);
            break;

         default:          
         Serial1.write(input);
      }
      bytesTransmitted++;
   }
}