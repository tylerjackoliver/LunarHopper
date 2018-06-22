//
//  Telemetry.cpp
//  Lunar Hopper
//
//  Created by Shane Lawson on 10/03/2018.
//

#include "Arduino.h"
#include "Telemetry.h"

#define START_DELIMETER 0x7E
#define ESCAPE_CHAR 0x7D
#define LENGTH 0x0F
#define FRAME_TYPE 0x10 //transmit request
#define FRAME_ID 0x00 //0 = no response, 1 = response
#define SERIAL_CHANNEL Serial1

Message::Message() {
   statusByte = 0x00;
}

void Message::setStatusByte(unsigned char inputByte) {
   statusByte = inputByte;
}

/*=====================*/
Recipient::Recipient() {
   SERIAL_CHANNEL.begin(115200, SERIAL_8N2);
   sumOfAddressBytes = 0x00;
   for (int i = 0; i < 10; i++)
   {
      addresses[i] = 0x00;
   }
}

Recipient::Recipient(unsigned long address64Upper, unsigned long address64Lower, unsigned int address16) {
   SERIAL_CHANNEL.begin(115200, SERIAL_8N2);
   this->setAddresses(address64Upper, address64Lower, address16);
}

void Recipient::setAddresses(unsigned long address64Upper, unsigned long address64Lower, unsigned int address16) {
   this->setAddress64(address64Upper, address64Lower);
   this->setAddress16(address16);
}

void Recipient::setAddress64(unsigned long address64Upper, unsigned long address64Lower) {
   unsigned char *ptr = NULL;
   ptr = (unsigned char *)&address64Lower;
   for (int i = 7; i >= 4; i--)
   {
      addresses[i] = *ptr;
      ptr++;
   }
   ptr = (unsigned char *)&address64Upper;
   for (int i = 3; i >= 0; i--)
   {
      addresses[i] = *ptr;
      ptr++;
   }

   this->sumAddressBytes();
}

void Recipient::setAddress16(unsigned int address16) {
   unsigned char *ptr = NULL;
   ptr = (unsigned char *)&address16;
   for (int i = 9; i >= 8; i--)
   {
      addresses[i] = *ptr;
      ptr++;
   }

   this->sumAddressBytes();
}

void Recipient::sumAddressBytes() {
   sumOfAddressBytes = 0x00;
   unsigned char *ptr = addresses;
   for (int i = 0; i < 10; i++)
   {
          sumOfAddressBytes += *ptr;
          ptr++;
   }
}

void Recipient::send(Message message) {
   SERIAL_CHANNEL.write(START_DELIMETER);
   SERIAL_CHANNEL.write(0x00);              //byte 0 of length
   SERIAL_CHANNEL.write(LENGTH);            //byte 1 of length
   SERIAL_CHANNEL.write(FRAME_TYPE);              
   SERIAL_CHANNEL.write(FRAME_ID);        

   unsigned char *ptr = addresses;
   for (int i = 0; i < 10; i++)
   {
      this->sendByte(*ptr);                     //address bytes
      ptr++;
   }

   SERIAL_CHANNEL.write(0x00);              //broadcast radius
   SERIAL_CHANNEL.write(0x00);              //options

   unsigned char sumMessageBytes = 0x00;
   ptr = (unsigned char *)&message;
   for (int i = 0; i < 1; i++)
   {
      this->sendByte(*ptr);                     //message bytes
      sumMessageBytes += *ptr;
      ptr++;
   }

   unsigned char sumBytes = FRAME_TYPE + FRAME_ID + sumOfAddressBytes + sumMessageBytes;
   this->sendByte(0xFF - sumBytes);  //checksum
}

void Recipient::sendByte(unsigned char input){
   switch(input){
      case 0x7E:
         SERIAL_CHANNEL.write(ESCAPE_CHAR);
         SERIAL_CHANNEL.write(0x5E);
         break;
      case 0x7D:
         SERIAL_CHANNEL.write(ESCAPE_CHAR);
         SERIAL_CHANNEL.write(0x5D);
         break;
      case 0x11:
         SERIAL_CHANNEL.write(ESCAPE_CHAR);
         SERIAL_CHANNEL.write(0x31);
         break;
      case 0x13:
         SERIAL_CHANNEL.write(ESCAPE_CHAR);
         SERIAL_CHANNEL.write(0x33);
         break;
      default:
         SERIAL_CHANNEL.write(input);
         break;
      }
}