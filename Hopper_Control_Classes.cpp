#include "Arduino.h"
// Classes
class Accelerometer {
  public:

  	// Initailises the pins
    setPins(int pinX, int pinY, int pinZ) {
      PIN_X = pinX;
      PIN_Y = pinY;
      PIN_Z = pinZ;
    }

    // Calculates the value of the stored angles
    void calcAngles() {
      const float ACCEL_SENSITIVITY = 0.8;
      float zeroVoltZ = 1.775;

      float x_g = ((analogRead(PIN_X) * VCC / 1023.00) - zeroVoltX) / ACCEL_SENSITIVITY;
      float y_g = ((analogRead(PIN_Y) * VCC / 1023.00) - zeroVoltY) / ACCEL_SENSITIVITY;
      float z_g = ((analogRead(PIN_Z) * VCC / 1023.00) - zeroVoltZ) / ACCEL_SENSITIVITY;
      float c_g = sqrt(sq(y_g) + sq(z_g));  //calculate the gravity using y and z axis
      float d_g = sqrt(sq(x_g) + sq(z_g));  // same as above using x and y axis
      float accAngleInRadiansX = -atan2 (x_g, c_g);
      float accAngleInRadiansY = -atan2 (y_g, d_g);
      angleX = accAngleInRadiansX * 180 / M_PI;  //get x-angle from accelerometer in degrees
      angleY = accAngleInRadiansY * 180 / M_PI;  //get y-angle from accelerometer in degrees
    }


    // Rests the stored angles
    void reset() {
   	  calibrated = false;
      acquisitionsCount = 0;
      voltageSumX = 0;
      voltageSumY = 0;
    }

    // Calcuatle the final variables
    void finaliseCalibration() {
      zeroVoltX = (voltageSumX * VCC) / (acquisitionsCount * 1023);
      zeroVoltY = (voltageSumY * VCC) / (acquisitionsCount * 1023);
      calibrated = true;
    }

    // Add one set of calibration numbers
    void addCalibrationValue() {
      voltageSumX += analogRead(PIN_X);
      voltageSumY += analogRead(PIN_Y);
      acquisitionsCount++;
    }

    // Getter for calibrated
    bool isCalibrated() {
      return calibrated;
    }

    // Getter for angleX
    float getAngleX() {
      return angleX;
    }

    // Getter for angleY
    float getAngleY() {
      return angleY;
    }

  private:
    const float VCC = 5.0;
        
    bool calibrated = false;

    int PIN_X;
    int PIN_Y;
    int PIN_Z;

    int acquisitionsCount;

    float voltageSumX;
    float voltageSumY;

    float angleX;
    float angleY;

    float zeroVoltX;
    float zeroVoltY;
};


class Gyro {
  public:

    setPins(int pinX, int pinY) {
      PIN_X = pinX;
      PIN_Y = pinY;
    }

    void calcAngles() {
      const float GYRO_SENSITIVITY = 0.04;

      int sampling_count = 0; 
      float sumX = 0.0; 
      float sumY = 0.0;
      // Get reading ten times-over sampling to reduce noise
      for (sampling_count = 0;sampling_count < 10; sampling_count++) {
        sumX += analogRead(PIN_X); // Add current Z gyro voltage to running total
        sumY += analogRead(PIN_Y);
      }

      rateX = ( (sumX * VCC) / (sampling_count * 1023.) - xGyro0V) / GYRO_SENSITIVITY * (1.0); // Calculate if any movement has occured in the X axis and correct angle (change sign due to how the gyro is mounted on the frame)
      rateY = ( (sumY * VCC) / (sampling_count * 1023.) - yGyro0V) / GYRO_SENSITIVITY * (1.0); //same for y axis
      angleX = rateX * (millis() - timeAcs + 15.00) * 0.001;  //get x angle from x-axis gyro
      angleY = rateY * (millis() - timeAcs + 15.00) * 0.001;  //get y angle from y-axis gyro
    }

    void reset() {
      calibrated = false;
      acquisitionsCount = 0;
      voltageSumX = 0;
      voltageSumY = 0;
    }

    void finaliseCalibration() {
      xGyro0V = (voltageSumX * VCC) / (acquisitionsCount * 1023);
      yGyro0V = (voltageSumY * VCC) / (acquisitionsCount * 1023);
      calibrated = true;
    }


    void addCalibrationValue() {
      voltageSumX += analogRead(PIN_X);
      voltageSumY += analogRead(PIN_Y);
      acquisitionsCount++;
    }

    float getAngleX() {
      return angleX;
    }

    float getAngleY() {
      return angleY;
    }

    float getRateX(){
      return rateX;
    }

    float getRateY(){
      return rateY;
    }
    
    void setTimeAcs(unsigned long time){
      timeAcs = time;
    }

  private:

    const float VCC = 5.0;
        
    int PIN_X;
    int PIN_Y;

    bool calibrated = false;

    int acquisitionsCount;

    float voltageSumX;
    float voltageSumY;

    float xGyro0V;
    float yGyro0V;
    
    float angleX;
    float angleY;

    float rateX;
    float rateY;

    unsigned long timeAcs = 0;
};
