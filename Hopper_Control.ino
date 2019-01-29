/*-------------------------------

Lunar Hopper Control System Code

Written by: Marian Daogaru, mn2g12

Updated by: Samuel Rowbotham      sjr1g18
            Jack Tyler            jt6g15
            Duncan Hamill         dh2g15
            Boateng Opoku-Yeboah  boy1g15
            Shane Lawson          sdl1n17
---------------------------------
*/

#include "Hopper_Control_Classes.cpp"

/*
 * DECLERATIONS
 */

/* PIN NUMBERS
 * Set pins for various input and output devices on arduino. e.g.- gyro, accelerometer,pressure transducer and solenoid valves.
 */
//Sensors (INPUTS)
const int PRES_TRANS_PIN = A4;  //Input from Pressure Transducer [ANALOG]
const int GYRO_PIN_X = A0;      //x-axis Gyro is connected to analog pin
const int GYRO_PIN_Y = A5;      //y-axis gyro
const int ACCEL_PIN_X = A1;     //x-axis accelerometer pin
const int ACCEL_PIN_Y = A2;     //y-axis accelerometer pin
const int ACCEL_PIN_Z = A3;     //z-axis accelerometer pin

// Actuators (OUTPUTS)
const int ACT_PIN_X_POS = 6;    //ACS valve positive x_axis
const int ACT_PIN_X_NEG = 5;    //ACS valve negative x_axis
const int ACT_PIN_Y_POS = 4;    //ACS valve positive y_axis
const int ACT_PIN_Y_NEG = 3;    //ACS valve negative y_axis
const int SOL_PIN_VENT = 12;    //Output to VENT solenoid control (SV2)
const int SOL_PIN_PRES = 10;    //Output to PRESSURE solenoid control (SV1)
const int SOL_PIN_OX = 11;      //Output to OX solenoid control(SV3)

// Control pins (INPUTS)
const int START_PIN = 30;
const int ABORT_PIN = 2;
const int DISARM_PIN = 1; // ADD
const int THROTTLE_PIN = A6;

// Valve constants for readability
const int VALVE_OPEN = HIGH;
const int VALVE_CLOSE = LOW;

/*
 * ACS
 */

// ACS configuration
unsigned long timeZG = 0;                      //Variable to record the start time of zeroing the gyros
unsigned long timeAcs = 0;                     //Variable used to record the start time of each ACS() loop
bool acsInCalibration = false;                 //Tracking variable to detect if ACS subsystem is active
bool acsCalibrated = false;                    //Tracking variable to record if ACS system has been acsCalibrated
bool midCycle = false;                         //Tracking variable to record if ACS system is mid Cycle

/*
 *  Main Engine
 */
// General
const float IGNITION_PRESSURE = 21.0; // bar (abs)
bool pressurised = false; // Records if the engine has been pressurised
bool ignited = false; // Records if the engine has been ignited

// oxPulse specific
int blipMode = 0;                            // Records the current mode of the oxPulse function (0: Ready to fire another blip, 1: Just fired a blip, 2: Just rested after a blip)
int oxPulseNo = 0;                           // Records the number of oxygen pulses completed (In Ignition)
unsigned long timePulse = 0;                 // Records the start time of the last ox blip


/*
 * System status
 * Used to record the current state of the hopper
 */
bool startStatus = false;                   // Records the state of the start pin to start the flight
bool disarmStatus = false;                  // Stores if the hopper has been disarmed
bool abortStatus = false;                   // Stores if the hopper has aborted
double currentPressure = 0;                 // Current pressre of the pressure transducer

// Initialisation of objects.
Accelerometer accelerometer;                // The accelerometers
Gyro gyro;                                  // The gyros
/*
 * CODE
 */

// Setup Loop
void setup() {
  Serial.begin(9600);
  Serial.println("Starting serial process...");
  pinMode(START_PIN, INPUT);      // Defines the Start pin as an INPUT
  pinMode(ABORT_PIN, INPUT);      // Defines the Abort pin as an INPUT
  pinMode(DISARM_PIN, INPUT);     // Defines the Disarm pin as an INPUT
  pinMode(THROTTLE_PIN, INPUT);   // Defines the Throttle as an INPUT
  pinMode(PRES_TRANS_PIN, INPUT); // Defines the Pressure Transducer reading as an INPUT
  pinMode(GYRO_PIN_X, INPUT);     // Defines the Gyro X reading as an INPUT
  pinMode(GYRO_PIN_Y, INPUT);     // Defines the Gyro Y reading as an INPUT
  pinMode(ACCEL_PIN_X, INPUT);    // Defines the Accelerometer X reading as an INPUT
  pinMode(ACCEL_PIN_Y, INPUT);    // Defines the Accelerometer Y reading as an INPUT
  pinMode(ACCEL_PIN_Z, INPUT);    // Defines the Accelerometer Z reading as an INPUT
  pinMode(ACT_PIN_X_POS, OUTPUT); // Defines the ACT positive X as an OUTPUT
  pinMode(ACT_PIN_X_NEG, OUTPUT); // Defines the ACT negative X as an OUTPUT
  pinMode(ACT_PIN_Y_POS, OUTPUT); // Defines the ACT positive Y as an OUTPUT
  pinMode(ACT_PIN_Y_NEG, OUTPUT); // Defines the ACT negative Y as an OUTPUT
  pinMode(SOL_PIN_VENT, OUTPUT);  // Defines the signal to Vent control as an OUTPUT
  pinMode(SOL_PIN_PRES, OUTPUT);  // Defines the signal to Pressure control as an OUTPUT
  pinMode(SOL_PIN_OX, OUTPUT);    // Defines the signal to OX control as an OUTPUT

  accelerometer.setPins(ACCEL_PIN_X,ACCEL_PIN_Y,ACCEL_PIN_Z);
  gyro.setPins(GYRO_PIN_X,GYRO_PIN_Y);
}

/*
 * Main execution loop
 */

void loop() {
  Serial.print("In loop:   ");
  Serial.println(timeZG);

  if (abortStatus == false) {
    Serial.println("ABORT status = false");
    if (disarmStatus == false) {
      if (startStatus == true) {
        Serial.println("Start = true");

        if (acsCalibrated == false) {
          Serial.println("Performing calibration...");
          acsCalibration();

        } else if (ignited == false) {
          Serial.print("oxPulseNo = ");
          Serial.println(oxPulseNo);
          oxPulse();
          
        } else {
          Serial.println("acs...");
          acs();
          Serial.println("throttle...");
          throttle();

        }
      } else {
        startStatus = digitalRead(START_PIN);
        Serial.println("Reading start pin...");

      }
      if (disarmStatus == false){ // Prevents disarm status from being overwritten
        disarmStatus = digitalRead(DISARM_PIN);

      }
      Serial.print("disarm status = ");
      Serial.println(disarmStatus);
    } else {
      disarm();

    }
  } else {
    abort();

  }
  if (abortStatus == false){ // Prevents abort status from being overwritten
      abortStatus = digitalRead(ABORT_PIN);

  }
  Serial.print("abort status = ");
  Serial.println(abortStatus);
}
  

/*
 * UPDATED TO LATEST VERSION 20/11/18 by Samuel Rowbotham
 *  Not Verified through system tests/simulations.
 *  Will slowly disarm the vehicle by having a gap between meco and ventAct
 */
void disarm() {
  const int ACT_VENT_DELAY = 5000; // 5 seconds between engine cut off and venting of act 
  disarmStatus = true;
  
  meco();

  delay(ACT_VENT_DELAY);
  ventAct();
}

/*
 * UPDATED TO LATEST VERSION 20/11/2018 by Samuel Rowbotham
 * Not Verified through system tests/simulations.
 * Instantly shutsoff the engine and vents the ACT
   The codes were therefore tested using the simulation function in the Xojo App which simulates the MECO remote being on/off and the hopper being armed/disarmed.
 */

void abort() { // Rename to abort for visual
  Serial.println("ABORTING");
  abortStatus = true;
  
  meco();
  ventAct();
}

// Vents the ACS
void ventAct() {
  Serial.println("Venting ACT");
  digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
  digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);
  digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
  digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);
}

void meco() {
  digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
  digitalWrite(SOL_PIN_VENT, VALVE_OPEN);
}

/*acsCheck()--checks the commands related to the ACS and runs the appropriate routine.
 *-------------Uses sub functions- ACS and acsCalibration
 *-------------Also performs tasks relating to venting and ACT test.
 *acsCalibration()-- Performs calibration for given calibration time
 *acs()--The main function housing the control law for the hopper.
 *-------Calculates current angles from gyro and accelerometers, applies complementary filtering
 *-------Uses the control law to regulate the ACT solenoid valves
 *
 *
 *Original Author: Zhihan Ma
 *Modified, updated and verified by Achal Mittal
 *NOT to be changed without approval from both Zhihan Ma and Achal Mittal
 */

void acsCalibration() {
  const unsigned long  CALIBRATION_TIME = 40000; // Time used to zero the gyro

  if (acsInCalibration == false){
    acsCalibrated = false;
    accelerometer.reset();
    gyro.reset();
    acsInCalibration = true;
    timeZG = millis();      //start zeroing time
    Serial.print("Checking, timeZG = ");
    Serial.println(timeZG);
  }
  
  if (timeZG + CALIBRATION_TIME > millis()) { // If calibration time is not over
    accelerometer.addCalibrationValue();
    gyro.addCalibrationValue();
    Serial.print("Still calibrating at elapsed time: ");
    Serial.println(timeZG + CALIBRATION_TIME-millis());

  } else {
    if (acsCalibrated == false) {  // If calibration time has just finished
      accelerometer.finaliseCalibration(); // Calculate the zero voltages for the accelerometer
      gyro.finaliseCalibration(); // Calculate the zero voltage for the gyroscope
      acsCalibrated = true;      // Record calibration has finished
    }
  }
}

// Calculates the angle of the hopper and activates the ACT in response. If the angle is greter then MAX_ANGLE it abort's.
void acs() {
  const int MAX_ANGLE = 15; // this angle will trigger abort
  const unsigned long TIME_INTERVAL = 25; //Time in milliseconds between each adjustment of the ACS system
  const float CFF_VECTOR_GYRO = 0.90; //complementary filter- choose what percentage of gyro angle you want in the final angle.
  const float CFF_VECTOR_ACCEL = 0.10; //complementary filter- choose what percentage of accelerometer angle you want in the final angle.
  const float CONTROL_LAW_GAIN = 0.5; // gain of control law
  const float CONTROL_LAW_DEADBAND = 0.1; // threshold for control law (deadband) #maybe this is a bit too low, might require some changes


  if (midCycle == false) { // If midCycle starting for first time or has just finished
    midCycle = true; // Record the midCycle is starting
    timeAcs = millis(); // Record the start time of the midCycle
    gyro.setTimeAcs(timeAcs);

  } else if (timeAcs + TIME_INTERVAL < millis()) { // If time interval has passed
    midCycle = false;
    
    // Calculate angular rates and integrate for angle
    accelerometer.calcAngles();
    gyro.calcAngles();

    float currentAngleX = CFF_VECTOR_GYRO * gyro.getAngleX() + CFF_VECTOR_ACCEL * accelerometer.getAngleX(); // Apply complementary filtering in z axis
    float currentAngleY = CFF_VECTOR_GYRO * gyro.getAngleY() + CFF_VECTOR_ACCEL * accelerometer.getAngleY(); // Apply complementary filtering in y axis

    float nextAngleX = currentAngleX + CONTROL_LAW_GAIN * gyro.getRateX();
    float nextAngleY = currentAngleY + CONTROL_LAW_GAIN * gyro.getRateY();

    Serial.print("NextAngleX = ");
    Serial.println(abs(nextAngleX));
    if (abs(nextAngleX) < MAX_ANGLE) {
      if (abs(nextAngleX) >= CONTROL_LAW_DEADBAND) {
        if (nextAngleX > 0) {  //control law
          digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
          digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);

        } else if (nextAngleX < 0) {
          digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
          digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);

        }
      } else {
        digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
        digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);

      }      
    } else {
      abort();
    }

    Serial.print("NextAngleY = ");
    Serial.println(abs(nextAngleY));
    if (abs(nextAngleY) < MAX_ANGLE) {
      if (abs(nextAngleY) >= CONTROL_LAW_DEADBAND) {
        if (nextAngleY > 0) {    // control law
          digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
          digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);

        } else if (nextAngleY < 0) {
         digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
         digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);

        
        } else {
          digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
          digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);

        }
      }
    } else {
      abort();
    }
  }
}


// Blips the OX to warm up the catalyst bed (Effective igniton)
void oxPulse() {
  const unsigned long BLIP_DURATION = 300; // Time in milliseconds the ox pulse lasts
  const unsigned long BLIP_REST_DURATION = 5000; // Duration for which the system will wait for heat to dissipare in the fuel
  const int OX_NO_PULSES = 3;

  pressurisation();

  if (pressurised == true) {
    if (blipMode == 0) { // Ready to fire another blip
      timePulse = millis();
      digitalWrite(SOL_PIN_OX, VALVE_OPEN);
      blipMode = 1;

    } else if (blipMode == 1 && timePulse + BLIP_DURATION < millis()) { // Just fired a blip
      digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
      blipMode = 2;
      timePulse = millis();

    } else if (blipMode == 2 && timePulse + BLIP_REST_DURATION < millis()) { // Just rested after a blip
      blipMode = 0;
      oxPulseNo++;
    }
  }
  if (oxPulseNo < OX_NO_PULSES) {
    ignited = true;
  }
}

// Pressurises the solonoid prior to oxPulse (ignition)
void pressurisation() {
  currentPressure = getPressure();

  Serial.print("Current pressure = ");
  Serial.println(currentPressure);

  if (currentPressure < IGNITION_PRESSURE) {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    pressurised = false;

  } else {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    pressurised = true;
  }
}

// Throttle control of the vehicle (controled manually)
void throttle() {
  currentPressure = getPressure();

  float throttlePressure = ((analogRead(THROTTLE_PIN) / 1023 * 21 )) + 19;
  Serial.print("Throttle pressure = ");
  Serial.println(throttlePressure);
  Serial.print("Current pressure = ");
  Serial.println(currentPressure);

  if (currentPressure < IGNITION_PRESSURE ) {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    Serial.println("current < ig");

  } else if (throttlePressure >= currentPressure && currentPressure > IGNITION_PRESSURE) {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    Serial.println("throttle >= current && current > ig");

  } else if (throttlePressure < IGNITION_PRESSURE && IGNITION_PRESSURE < currentPressure) {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    Serial.println("throttle < ig < current");

  } else if (throttlePressure < currentPressure && IGNITION_PRESSURE < currentPressure && IGNITION_PRESSURE < throttlePressure) {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    Serial.println("ig < throttle < current");

  }
}

// Returns the pressure in bars of the Pressure Transducer
double getPressure() {
  int voltage = analogRead(PRES_TRANS_PIN);
  return voltage * (40 / 1023); //int * (bar/int) returns: bar
}

// Not used currently Resets all values
void reset() {
  timeZG = 0;
  acsInCalibration = false;
  acsCalibrated = false;
  midCycle = false;
  timeAcs = 0;
  blipMode = 0;
  oxPulseNo = 0;
  timePulse = 0;
  currentPressure = 0;
  pressurised = false;
  startStatus = false;
  disarmStatus = false;
  abortStatus = false;
}