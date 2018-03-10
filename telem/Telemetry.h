//
//  Telemetry.h
//  Lunar Hopper
//
//  Created by Shane Lawson on 10/03/2018.
//

#ifndef TELEMETRY_H
#define TELEMETRY_H

class Address64
{
 public:
   Address64();
   const char* getAddressPtr()const;

 private:
   char addressBytes[8];
};

class Message
{
 public:
   Message();
   void setContent(const char *, int);
   void sendTo(const Address64);

 private:
   static const int MAX_BYTES = 54; //with current configuration, max transmit length in bytes without fragmentation
   static const int MAX_CONTENT_BYTES = 30; //I think acutally 36, but to be safe
   int bytesTransmitted;
   int contentLength;
   char lengthBytes[2];
   char frameIDBytes[2];
   char address16[2];
   char other[2];
   char content[MAX_CONTENT_BYTES];
   char checksum;
   void sendByte(char);
   void calcChecksumAndLength(const Address64);
};

#endif