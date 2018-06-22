//
//  Telemetry.h
//  Lunar Hopper
//
//  Created by Shane Lawson on 10/03/2018.
//

#ifndef TELEMETRY_H
#define TELEMETRY_H

class Message {
   friend class Recipient;
   
   public:
     Message();
     void setStatusByte(unsigned char);

   private:
     unsigned char statusByte;
};

class Recipient {
   public:
     Recipient();
     Recipient(unsigned long, unsigned long, unsigned int);
     void setAddresses(unsigned long, unsigned long, unsigned int);
     void setAddress64(unsigned long, unsigned long);
     void setAddress16(unsigned int);
     void send(Message);

   private:
     unsigned char addresses[10];
     unsigned char sumOfAddressBytes;
     void sumAddressBytes();
     void sendByte(unsigned char);
};

#endif