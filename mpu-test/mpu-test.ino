#include <I2Cdev.h>
#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const int ACT_PIN_X_POS = 6;    //ACS valve positive x_axis
const int ACT_PIN_X_NEG = 5;    //ACS valve negative x_axis
const int ACT_PIN_Y_POS = 4;    //ACS valve positive y_axis
const int ACT_PIN_Y_NEG = 3;    //ACS valve negative y_axis

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

   intitializeACSPins();
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

void intitializeACSPins() {
   pinMode(ACT_PIN_X_POS, OUTPUT);
   pinMode(ACT_PIN_X_NEG, OUTPUT);
   pinMode(ACT_PIN_Y_POS, OUTPUT);
   pinMode(ACT_PIN_Y_NEG, OUTPUT);
   String thrusters[] = {"back", "front", "left", "right"};

   Serial.print("Testing thrusters: ");

   for (int i = 3; i < 7; i++)
   {
     Serial.print(thrusters[i - 3] + "...");
     digitalWrite(i, HIGH);
     delay(200);
     digitalWrite(i, LOW);
     delay(200);
   }
   Serial.println("Done!");
}