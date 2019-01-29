#include "Arduino.h"
// Classes
class Accelerometer {
  public:

  	// Sets the pins of the accelerometer
    setPins(int pinX, int pinY, int pinZ) {
      PIN_X = pinX;
      PIN_Y = pinY;
      PIN_Z = pinZ;
    }

    // Calculates the current angle of the accelerometer
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


    // Resets the accelerometer
    void reset() {
   	  calibrated = false;
      acquisitionsCount = 0;
      voltageSumX = 0;
      voltageSumY = 0;
    }

    // Calculate the final variables
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
    const float VCC = 5.0; // Stores the base voltage of the VCC

    int PIN_X; // Stores the accelerometer's X pin
    int PIN_Y; // Stores the accelerometer's Y pin
    int PIN_Z; // Stores the accelerometer's Z pin

    bool calibrated = false; // Stores if the accelerometer is calibrated

    int acquisitionsCount; // Number of calibration values taken

    float voltageSumX; // Sum of accelerometers X calibration Values
    float voltageSumY; // Sum of accelerometers Y calibration Values

    float zeroVoltX; // Calibrated zero voltage of the accelerometer X
    float zeroVoltY; // Calibrated zero voltage of the accelerometer Y
    
    float angleX; // Accelerometers current angle in the x axis
    float angleY; // Accelerometers current angle in the Y axis
};


class Gyro {
  public:

    // Sets the pins of the gyro
    setPins(int pinX, int pinY) {
      PIN_X = pinX;
      PIN_Y = pinY;
    }

    // Calculates the current angles of the gyros and their rate of change
    void calcAngles() {
      const float GYRO_SENSITIVITY = 0.04;

      int samplingCount = 0; 
      float sumX = 0.0; 
      float sumY = 0.0;
      // Get reading ten times-over sampling to reduce noise
      for (samplingCount = 0;samplingCount < 10; samplingCount++) {
        sumX += analogRead(PIN_X); // Add current Z gyro voltage to running total
        sumY += analogRead(PIN_Y);
      }

      rateX = ( (sumX * VCC) / (samplingCount * 1023.) - zeroVoltX) / GYRO_SENSITIVITY * (1.0); // Calculate if any movement has occured in the X axis and correct angle (change sign due to how the gyro is mounted on the frame)
      rateY = ( (sumY * VCC) / (samplingCount * 1023.) - zeroVoltY) / GYRO_SENSITIVITY * (1.0); // Same for y axis
      angleX = rateX * (millis() - timeAcs + 15.00) * 0.001;  // Get x angle from x-axis gyro
      angleY = rateY * (millis() - timeAcs + 15.00) * 0.001;  // Get y angle from y-axis gyro
    }

    // Resets the gyro
    void reset() {
      calibrated = false;
      acquisitionsCount = 0;
      voltageSumX = 0;
      voltageSumY = 0;
    }

    // Calibrates the zeroVoltage of its X and Y angles
    void finaliseCalibration() {
      zeroVoltX = (voltageSumX * VCC) / (acquisitionsCount * 1023);
      zeroVoltY = (voltageSumY * VCC) / (acquisitionsCount * 1023);
      calibrated = true;
    }

    // Adds a calibration value to the gyro's sum
    void addCalibrationValue() {
      voltageSumX += analogRead(PIN_X);
      voltageSumY += analogRead(PIN_Y);
      acquisitionsCount++;
    }

    // Getter for angleX
    float getAngleX() {
      return angleX;
    }

    // Getter for angleY
    float getAngleY() {
      return angleY;
    }

    // Getter for rateX
    float getRateX(){
      return rateX;
    }

    // Getter for rateY
    float getRateY(){
      return rateY;
    }
    
    // Sets the last time the ACS corrected the hoppers orientation
    void setTimeAcs(unsigned long time){
      timeAcs = time;
    }

  private:

    const float VCC = 5.0; // Stores the base voltage of the VCC
        
    int PIN_X; // Gyro X pin
    int PIN_Y; // Gyro Y pin

    bool calibrated = false; // Stores if the gyro is calibrated

    int acquisitionsCount; // Number of calibration values taken

    float voltageSumX; // Sum of gyro X calibration Values
    float voltageSumY; // Sum of gyro Y calibration Values

    float zeroVoltX; // Calibrated zero voltage of the gyro X
    float zeroVoltY; // Calibrated zero voltage of the gyro Y
    
    float angleX; // Gyros current angle in the x axis
    float angleY; // Gyros current angle in the Y axis

    float rateX; // Gyros current rate of change in the x axis
    float rateY; // Gyros current rate of change in the y axis

    unsigned long timeAcs = 0; // The last time the ACS corrected the hoppers orientation
};