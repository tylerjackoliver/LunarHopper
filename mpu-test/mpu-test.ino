#include <I2Cdev.h>
#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
#endif

   Serial.begin(9600);

   Serial.print("Connecting to MPU...");
   accelgyro.initialize();

   if (accelgyro.testConnection())
   {
      Serial.println("connection successful!");
   }
   else
   {
      Serial.println("Sad day :( connection unsuccessful!");
   }
}

void loop()
{
   accelgyro.getAcceleration(&ax, &ay, &az);
   accelgyro.getRotation(&gx, &gy, &gz);
   // ax = ax / 8192;
   // ay = ay / 8192;
   // az = az / 8192;
   Serial.print("a/g:\t");
   Serial.print(ax);
   Serial.print("\t");
   Serial.print(ay);
   Serial.print("\t");
   Serial.print(az);
   Serial.print("\t");
   Serial.print(gx);
   Serial.print("\t");
   Serial.print(gy);
   Serial.print("\t");
   Serial.println(gz);
   delay(500);

}
