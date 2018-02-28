/*-------------------------------

Lunar Hopper Control System Code

Written by: Marian Daogaru, mn2g12

Updated by: Jack Tyler, jt6g15, Duncan Hamill, dh2g15, Boateng Opoku-Yeboah, boy1g15

---------------------------------
*/


/*
 * PIN NUMBERS
 * Set pins for various input and output devices on arduino. e.g.- gyro, accelerometer,pressure transducer and solenoid valves.
 */
// Valve constants (just for a smidge more code readability)
const int VALVE_OPEN = HIGH;
const int VALVE_CLOSE = LOW;

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

/*Gyro + Accelerometer setup*/
float Vcc = 5.0;//Gyro is running at 5V from Arduino
float gyroSensitivity = 0.04;//gyro xensitivity at 40mV/deg/s
float z_accZeroVoltage = 1.775; //accelerometer z axis 0 voltage
float x_accZeroVoltage = 0; //accelerometer x axis 0 voltage
float y_accZeroVoltage = 0; //accelerometer y axis 0 voltage

/*ACS configuration*/
float a = 0.5;// gain of control law
float k = 0.1;// threshold for control law (deadband) #maybe this is a bit too low, might require some changes !!!
float XGyro0V;//variable to store the x -axis gyro's 0 (no rotation) voltage
float YGyro0V;//variable to store the y-axis gyro's 0 (no rotation) voltage
float xRate = 0;//initialise x-axis gyro rate
float yRate = 0;//initialise y-axis gyro rate
unsigned long TimeACS = 0; //Variable used to record the start time of each ACS(); loop
unsigned long  CalibrationTime = 40000;//Time used to zero the gyro
unsigned long acquisitions_count = 0; //Variable used to record the number of cycles used for ACS calibration
float x_voltage_sum = 0; //x gyro voltage
float y_voltage_sum = 0;//y gyro voltage
float yacc_voltage_sum = 0;//y-axis acceleroometer voltage
float zacc_voltage_sum = 0;//z-axis accelerometer voltage
float xacc_voltage_sum = 0;//x-axis accelerometer voltage
float x_sum = 0;//sum of x-gyro readings used after calibration to reduce noise within the ACS(); function.
float y_sum = 0;//;//sum of y-gyro readings used after calibration to reduce noise within the ACS(); function.
const unsigned long Time_Interval = 25; //Time in milliseconds between each adjustment of the ACS system
bool Cycle = false; //Tracking variable to record if ACS system is mid cycle
bool wasOXon = false; //Variable to track if control law can be activated based on whether OX caommand has been given or not.
unsigned long sampling_count = 0;//used within ACS function in conjunction with z_sum and y_sum. records the no. of readings taken to reduce noise.
unsigned long TimeZG = 0; //Variable to record the start time of zeroing the gyros
bool ACSActive = false; //Tracking variable to detect if ACS subsystem is active
bool Calibrated = false; //Tracking variable to record if ACS system has been calibrated
float accSensitivity = 0.8;//accelerometer sensitivity
float acc_angleinradians_x;
float acc_angleinradians_y;
float cffvector_gyro = 0.90; //complementary filter- choose what percentage of gyro angle you want in the final angle.
float cffvector_acc = 0.10; //complementary filter- choose what percentage of accelerometer angle you want in the final angle.
int maxAngle = 15; // this angle will trigger MECO
/*
 * Flight configuration
 */

const unsigned long blipDuration = 300; // Time in milliseconds the ox pulse lasts
const float ignitionPressure = 21.0; // bar (abs)
const float initialFlightPressure = 21.0; // bar (abs)

/*
 * MECO configuration
 */
const unsigned long mecoMinHold = 1000; // Minimum duration of MECO

/*
 * LOSS OF SIGNAL configuration
 */
const float maxTilt = 15.0; // Maximum tilt angle allowed during LOS autopilot control. If exceeded, the Hopper is disarmed and the rocket killed. Valid for both pitch and yaw.
const float descentPressure = 24.5; // bar abs. This is the pressure demanded by the LOS autopilot control. Should be pressure for hover at dry hopper - 4 bar - Verify this
const unsigned long expFlightTime = 15000; // Expected flight time in milliseconds.

/*
 * Radio link configuration
 */

const unsigned long telemetryTxInterval = 20;
const unsigned long telemetryTxNetworkControl = 300;
const unsigned long telemetryRxMax = 3000;
/*
 * IMPORTANT NOTE: IF THE XBEE MODULES ON THE REMOTE OR THE PC ARE CHANGED,
 * THEIR ADDRESSES NEED TO BE UPDATED IN THE Telemetry.h LIBRARY FILE
 */


/*
 * ***** END OF CONFIGURATION *****
 * DO NOT EDIT THE FOLLOWING VARIABLES! THEY WILL BE MANAGED BY THE SYSTEM
 *
 */


/*
 * Command variables
 * Used to recored the incoming commands from the remote and PC interface
 * Initialise all to false.
 */

bool armCommand = false;
bool ACScommand = false;
bool ACTtestcommand = false;
bool ACTventcommand = false;
bool pressCommand = false;
bool pressIgnitionCommand = false;
bool oxCommand = false;
bool oxBlipCommand = false;
bool ventCommand = false;
bool mecoCommand = false;
double throttleValue = 0.0;

/*
 * System status
 * Used to record the current state of the hopper
 */
// Constant states
const int STATUS_OFF = 0;
const int STATUS_ON = 1;
const int STATUS_TEST = 2; // STATUS_TEST only used once, consider removing
/* Note: if STATUS_TEST is removed, can change these ints to bools, which makes 
 more logical sense in an ON/OFF perspective. I don't think any of the if 
 statements would require changing, since if (status[true] == STATUS_ON[true]) still works, 
 where bit in brackets is what it evaluates to. */

int armStatus = STATUS_OFF;
int ACSstatus = STATUS_OFF;
int ACTteststatus = STATUS_OFF; // !!! not found elsewhere
int ACTventstatus = STATUS_OFF;
int oxStatus = STATUS_OFF;
int ventStatus = STATUS_OFF;
int mecoStatus = STATUS_OFF;
// pressStatus does not follow the same as others,
// has an off, ignition pressure, and flight pressure
// consider changing how this behaves, or adding other
// STATUS constants for consistency across status variables
int pressStatus = STATUS_OFF;

/*
 * Data that will be sent to the PC for telemetry purposes
 */
double currentPressure = 0;
float currentangle_x = 0; //current angle after applying complementary filtering in the x axis.
float currentangle_y = 0; //current angle after applying complementary filtering in the y axis.
float acc_angle_x = 0;//accelerometer xaxis angle
float acc_angle_y = 0;//accelerometer y-axis angle
float yAngle = 0;//initialise y axis gyro angle
float xAngle = 0;//initialise z axis gyro angle
unsigned int rssi_pc = 0;
unsigned int rssi_remote = 0;



// Variables used for ventpressure
float PTVoltage = 0; //Variable used to store the voltage output from the pressure transducer

// Variables used for oxblip
unsigned long Timepulse = 0; //Variable used to record the start time of an ox blip
bool oxpulse = false; //Tracking variable used to record if oxpulse has been started

// Variables for MECO control
unsigned long mecoStart = 0;//start time of meco
bool wasMECO = false;//check if meco was on or off

// Variables for LOS control
unsigned long flightStart = 0;//variable to store flight time

// Variables for Start
int startStatus = STATUS_OFF;
int startPin = 30;
int mecoPin = 2;
const unsigned long blipRestDuration = 5000; //duration for which the system will wait for heat to dissipare in the fuel
int blipRest = 0;
int blipFire = 0;
int oxNoPulses = 3;
int oxPulseNo = 0;
const int throttlePin = A6;
float throttlePressure = 0;


/*
 * Setup loop
 */

void setup() {
  Serial.begin(9600);
  Serial.println("Starting serial process...");
  pinMode(PRES_TRANS_PIN, INPUT); //Define Pressure Transducer reading as an INPUT
  pinMode(GYRO_PIN_X, INPUT);//reads gyro input
  pinMode(GYRO_PIN_Y, INPUT);
  pinMode(ACT_PIN_X_POS, OUTPUT);
  pinMode(ACT_PIN_X_NEG, OUTPUT);
  pinMode(ACT_PIN_Y_POS, OUTPUT);
  pinMode(ACT_PIN_Y_NEG, OUTPUT);
  pinMode(SOL_PIN_VENT, OUTPUT); //Define signal to Vent control as an OUTPUT
  pinMode(SOL_PIN_PRES, OUTPUT); //Define signal to Pressure control as an OUTPUT
  pinMode(SOL_PIN_OX, OUTPUT); //Define signal to OX control as an OUTPUT
  pinMode(startPin, INPUT);
  pinMode(mecoPin, INPUT);
  pinMode(throttlePin, INPUT);

}

/*
 * Main execution loop
 */

void loop() {

  /*  if (armCommand == false){
      // System is DISARMED
      disarm();
    }
    else {
      // System is ARMED
      armStatus = STATUS_ON;

      // Don't listen if MECO has just been turned on
      if (mecoStart + mecoMinHold < millis())
      {
        if (mecoCommand == true) {
          // MECO is activated
          mecoStatus = STATUS_ON;
          meco_high();
        } else {
          // NORMAL OPERATION (MECO DOWN, ARM ON)
          wasMECO = false;
          mecoStatus = STATUS_OFF;
          ACS_check(); // Run ACS subsystem function
          pressurization(); // Run Vent/Pressure subsystem function
          ox_pulse(); // Run OX PULSE function
          thruster(); // Run THRUSTER function
          }
      }
    } */
  Serial.print("In loop:   ");
  Serial.println(TimeZG);
  if (mecoStatus == STATUS_OFF)
  {
    Serial.println("MECO status: OFF");
    if (startStatus == STATUS_ON)
    {
      Serial.println("Start = 1");
      if (ACSActive == false)
      {
        Serial.println("Performing ACS Check...");
        ACS_Check();
      }
      else if (Calibrated == false)
      {
        Serial.println("Performing calibration...");
        ACS_Calibration();
      }
      else if (oxPulseNo < oxNoPulses)
      {
        Serial.print("oxPulseNo = ");
        Serial.print(oxPulseNo);
        ox_Pulse();
      }
      else
      {
        Serial.println("Working...");
        ACS();
        throttle();
      }
    }

    else
    {
      startStatus = digitalRead(startPin);
      Serial.println("Reading start pin...");
    }
    mecoStatus = digitalRead(mecoPin);
    Serial.print(" MECO status = ");
    Serial.println(mecoStatus);
  }

  else
  {
    meco_high();
  }
}

/*
 * UPDATED TO LATEST VERSION 23/3/15 by Achal Mittal
 * Verified through system tests/simulations.
 * Will enable the systems on the hopper when activated
 */

void disarm() {

  armStatus = STATUS_OFF;
  pressStatus = STATUS_OFF;
  oxStatus = STATUS_OFF;
  ACSstatus = STATUS_OFF;
  ACTventstatus = STATUS_ON;
  ventStatus = STATUS_ON;
  Calibrated = false; // Reset the calibration tracking variable
  PTVoltage = analogRead(PRES_TRANS_PIN); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage * 39) / 1023) + 1; // Calculate pressure in system in bar (abs)
  digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
  digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  digitalWrite(SOL_PIN_VENT, VALVE_CLOSE); // Open the VENT solenoid
  digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
  digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);
  digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
  digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);
  if (mecoCommand == true) { // If MECO has been activated
    mecoStatus = STATUS_ON;
  } else { // If MECO is inactive
    mecoStatus = STATUS_OFF;
  }
}

/*
 * UPDATED TO LATEST VERSION 24/3/15 by Achal Mittal
 * Verified through system tests/simulations.
 * Will shut down the hopper systems if MECO is activated, but open the vent valve
 * Note: The MECO remote was never actually used. Neither in phase III nor in phase IV.
   The codes were therefore tested using the simulation function in the Xojo App which simulates the MECO remote being on/off and the hopper being armed/disarmed.
 */

void meco_high() {
  Serial.println("in MECO");
  mecoStatus = STATUS_ON;

  pressStatus = STATUS_OFF;
  oxStatus = STATUS_OFF;
  ACSstatus = STATUS_OFF;
  ACTventstatus = STATUS_ON;
  ventStatus = STATUS_ON;
  startStatus = STATUS_OFF;

  // Override all the radio commands so that when MECO is turned off, the system stays shut
  ACScommand = false;
  pressCommand = false;
  pressIgnitionCommand = false;
  blipFire = 0;
  blipRest = 0;
  ACTventcommand = true;
  ventCommand = true;
  throttleValue = 0;

  Calibrated = false; //Reset Calibrated variable to ensure ACS systm recalibrates when system restarts
  PTVoltage = analogRead(PRES_TRANS_PIN); //Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage * 39) / 1023) + 1; //Calculate pressure in system in bar (abs)
  digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
  digitalWrite(SOL_PIN_VENT, VALVE_OPEN);
  digitalWrite(ACT_PIN_X_POS, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN); // vent ACS system

  if (!wasMECO)
  {
    mecoStart = millis();
    wasMECO = true;
  }
}
/*ACS_check()--checks the commands related to the ACS and runs the appropriate routine.
 *-------------Uses sub functions- ACS and ACS_calibration
 *-------------Also performs tasks relating to venting and ACT test.
 *ACS_calibration()-- Performs calibration for given calibration time
 *ACS()--The main function housing the control law for the hopper.
 *-------Calculates current angles from gyro and accelerometers, applies complementary filtering
 *-------Uses the control law to regulate the ACT solenoid valves
 *
 *
 *Original Author: Zhihan Ma
 *Modified, updated and verified by Achal Mittal
 *NOT to be changed without approval from both Zhihan Ma and Achal Mittal
 */
void ACS_Check()
{
  if (ACSActive == false)
  { //if acs has just been activated, start calibration process
    Calibrated = false;
    y_voltage_sum = 0;
    x_voltage_sum = 0;
    yacc_voltage_sum = 0;//y-axis acceleroometer voltage
    zacc_voltage_sum = 0;//z-axis accelerometer voltage
    xacc_voltage_sum = 0;//x-axis accelerometer voltage
    acquisitions_count = 0;
    ACSActive = true;//indiicate acs is active
    TimeZG = millis();//start zeroing time
    ACS_Calibration(); // Calibrate ACS+sensors
    Serial.print(" Checking, Timezg = ");
    Serial.println(TimeZG);
  }
}

void ACS_check() {
  if (ACScommand == true) { //If ACS signal is high
    ACSstatus = STATUS_ON;

    if (ACSActive == false) { //if acs has just been activated, start calibration process
      Calibrated = false;
      y_voltage_sum = 0;
      x_voltage_sum = 0;
      yacc_voltage_sum = 0;//y-axis acceleroometer voltage
      zacc_voltage_sum = 0;//z-axis accelerometer voltage
      xacc_voltage_sum = 0;//x-axis accelerometer voltage
      acquisitions_count = 0;
      ACSActive = true;//indiicate acs is active
      TimeZG = millis();//start zeroing time
      ACS_Calibration();
    }//start calibration function
    else {
      if (Calibrated == false) { //until calibration time is over, keep calibrating
        ACS_Calibration();
      }

      else if (Calibrated == true) {
        ACS();//start the main ACS function after calibration is complete
      }
    }
  }
  //ACTtestcommand used before flight to check the ACS valves are working properly
  //The command will open two valves on each axis for 15 ms one after the other
  else if (ACTtestcommand == true) { //if ACTtestcommand signal is high
    int i;
    ACSstatus = STATUS_TEST;
    for (i = 0; i < 1; i++) { //run just once
      digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
      digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);
      delay(15);
      digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
      digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);
      digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
      digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);
      delay(15);
      digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
      digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);
    }
    ACTtestcommand = false;
  }
  else if (ACTventcommand == true) { //when ACTventcommand signal is high
    ACTventstatus = STATUS_ON; 
    //open all valves
    digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
    digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);
    digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
    digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);
  }

  else {
    ACSstatus = STATUS_OFF;
    ACTventstatus = STATUS_OFF;
    //close all valves
    digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
    digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);
    digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
    digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);
    ACSActive = false;//restart calibration process in next run

  }

}

void ACS_Calibration() {
  if (TimeZG + CalibrationTime > millis()) { // If calibration time is not over
    x_voltage_sum += analogRead(GYRO_PIN_X);
    y_voltage_sum += analogRead(GYRO_PIN_Y);
    xacc_voltage_sum += analogRead(ACCEL_PIN_X);
    yacc_voltage_sum += analogRead(ACCEL_PIN_Y);
    acquisitions_count++; // Increment the counter
    Serial.print("Still calibrating at elapsed time: ");
    Serial.println(TimeZG + CalibrationTime-millis());
  } else {
    if (Calibrated == false) {  // If calibration time has just finished
      YGyro0V = (y_voltage_sum * Vcc) / (acquisitions_count * 1023.00); //Calculate zero rate voltage for y gro
      XGyro0V = (x_voltage_sum * Vcc) / (acquisitions_count * 1023.00); // Calculate the zero rate voltage for the Z axis gyro
      x_accZeroVoltage = (xacc_voltage_sum * Vcc) / (acquisitions_count * 1023);
      y_accZeroVoltage = (yacc_voltage_sum * Vcc) / (acquisitions_count * 1023);
      Calibrated = true;// Record calibration has finished
      x_voltage_sum = 0;
      y_voltage_sum = 0;
      xacc_voltage_sum = 0;
      yacc_voltage_sum = 0; // Reset variable ready for next task
      acquisitions_count = 0; // Reset the counter

    }
  }
}

void ACS() {
  if (Cycle == false) { // If cycle starting for first time or has just finished
    TimeACS = millis(); // Record the start time of the cycle
    Cycle = true; // Record the cycle is starting
  }

  else if (TimeACS + Time_Interval < millis()) { // If time interval has passed
    int i;
    // Get reading ten times-over sampling to reduce noise
    for (i = 0; i < 10; i++) {
      y_sum += analogRead(GYRO_PIN_Y);
      x_sum += analogRead(GYRO_PIN_X); // Add current Z gyro voltage to running total
      sampling_count++; // Increment the counter
    }
    // Calculate angular rates and integrate for angles
    xRate = ( (x_sum * Vcc) / (sampling_count * 1023.) - XGyro0V) / gyroSensitivity * (1.0); // Calculate if any movement has occured in the X axis and correct angle (change sign due to how the gyro is mounted on the frame)
    yRate = ( (y_sum * Vcc) / (sampling_count * 1023.) - YGyro0V) / gyroSensitivity * (1.0); //same for y axis

    float x_g = ((analogRead(ACCEL_PIN_X) * Vcc / 1023.00) - x_accZeroVoltage) / accSensitivity;
    float y_g = ((analogRead(ACCEL_PIN_Y) * Vcc / 1023.00) - y_accZeroVoltage) / accSensitivity;
    float z_g = ((analogRead(ACCEL_PIN_Z) * Vcc / 1023.00) - z_accZeroVoltage) / accSensitivity;


    float c_g = sqrt(sq(y_g) + sq(z_g)); //calculate the gravity using y and z axis
    float d_g = sqrt(sq(x_g) + sq(z_g)); // same as above using x and y axis
    acc_angleinradians_x = -atan2 (x_g, c_g);
    acc_angleinradians_y = -atan2 (y_g, d_g);
    acc_angle_x = acc_angleinradians_x * 180 / M_PI; //get x-angle from accelerometer in degrees
    acc_angle_y = acc_angleinradians_y * 180 / M_PI; //get y-angle from accelerometer in degrees
    yAngle = yRate * (millis() - TimeACS + 15.00) * 0.001; //get y angle from y-axis gyro
    xAngle = xRate * (millis() - TimeACS + 15.00) * 0.001; //get x angle from x-axis gyro
    currentangle_x = cffvector_gyro * (xAngle) + cffvector_acc * acc_angle_x; //apply complementary filtering in z axis
    currentangle_y = cffvector_gyro * (yAngle) + cffvector_acc * acc_angle_y; //apply complementary filtering in y axis


    //Apply control law and regulate the solenoid valves only after oxcommand is on
    //if (oxCommand ==true){////change
    Serial.print("ANGLE = ");
    Serial.print(abs(currentangle_x + a * xRate));
    if (abs(currentangle_x + a * xRate) < maxAngle) {
      if (abs(currentangle_x + a * xRate) >= k) {
        if (currentangle_x + a * xRate > 0) {    //control law
          digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
          digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);


        }
        else if (currentangle_x + a * xRate < 0) {
          //Serial.println(currentangle+a*gyroRate);
          digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);
          digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);

        }
        else {
          digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);
          digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
        }
      }
      else if (abs(currentangle_y + a * yRate) >= k) {
        if (currentangle_y + a * yRate > 0) {    //control law
          digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
          digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);

        }
        else if (currentangle_y + a * yRate < 0) {

         digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);
         digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);

        }
        else {
          digitalWrite(ACT_PIN_Y_POS, VALVE_CLOSE);
          digitalWrite(ACT_PIN_Y_NEG, VALVE_CLOSE);
        }
      }

      Cycle = false;//reset cycle to false

      x_sum = 0; // Reset variable for next loop
      y_sum = 0;
      sampling_count = 0; // Reset the counter
      delay(10); //Cycles was 15 before, but now is 10. if 15ms, we want 50 Hz which is 20ms. Thus, 10 gives more room to maneouvre.
      //}
    }
    else {
      meco_high();
    }
  }
}

/*
 * Author: Achal Mittal
 * Modified and Verified by Achal Mittal on 23/3/15
 * Controls the venting and pressurization of the main propulsion system
 */

/* Original Author: N Zapponi
 * Updated to latest version 23/3/15 by Achal Mittal
 * Blips the OX to warm up the catalyst bed
 * As yet unverified.
 * Update (7/6/15): It was found that the code for blipping works when simulated with an LED but does not work for an actual solenoid valve. Modify/change as necessary.
 * Update (28/10/16): The above has been fixed but not verified.
*/
// void ox_pulse() {
//   if (oxBlipCommand == true && pressStatus == STATUS_ON) {
//     // If Blip command is active and system is pressurised for catalyst warm up
//     if (oxpulse == false) { // If oxpulse command received this loop
//       Timepulse = millis(); // Record start time of pulse
//       oxpulse = true; // Record the oxpulse command has been received
//       digitalWrite(SOL_PIN_OX, VALVE_OPEN);
//     } else {
//       if (Timepulse + blipDuration > millis()) { //If blip duration has passed
//         digitalWrite(SOL_PIN_OX, VALVE_OPEN);
//       } else {
//         digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
//         oxpulse = false; // Reset the tracking variable
//         oxBlipCommand = false;  // Cancel signal to enable blip effect
//       }
//     }
//   }
// }

/* 29/10/16, Jack Tyler: There seems to be ox_pulse() and ox_Pulse() -- the one below seems to be the main function, but this needs to be verified. */

void ox_Pulse() {
  PTVoltage = analogRead(PRES_TRANS_PIN); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  Pressurisation();

  if (pressStatus == STATUS_ON) {
    if (blipFire == 0 && blipRest == 0)
    {
      Timepulse = millis();
      digitalWrite(SOL_PIN_OX, VALVE_OPEN);
      blipFire = 1;
    }
    else if (blipFire == 1 && blipRest == 0 && Timepulse + blipDuration < millis())
    {
      digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
      blipFire = 0;
      blipRest = 1;
      Timepulse = millis();
    }
    else if (blipFire == 0 && blipRest == 1 && Timepulse + blipRestDuration < millis())
    {
      blipRest = 0;
      digitalWrite(oxPulseNo, VALVE_OPEN);
      oxPulseNo++;
    }
  }
}

// /*
//  * Modified and updated to latest version on 24/3/15 by Achal Mittal
//  * Verified to work in May 2015 by using LEDs and potentiometer(to simulate pressure transducer) and throttle.
//  * Controls the flow of ox to the rocket during flight
//  *
//  */
// void pressurization() {

//   PTVoltage = analogRead(PRES_TRANS_PIN); // Read the voltage from the pressure transducer
//   currentPressure = ((PTVoltage / 1023 * 8) + 1)  ; // Calculate pressure in system in bar (abs)

//   if (ventCommand == true) { // If vent signal is high
//     digitalWrite(SOL_PIN_VENT, VALVE_OPEN);
//     ventStatus = STATUS_ON;
//   }
//   else { // If vent signal is low
//     digitalWrite(SOL_PIN_VENT, VALVE_CLOSE);
//     ventStatus = STATUS_OFF;
//   }

//   if (pressIgnitionCommand == true) {
//     // IGNITION PRESSURE
//     // System set to lower pressure to warm up the catalyst bed
//     pressStatus = 1; // Record system set to deliver the ignition pressure
//     if (currentPressure < ignitionPressure)
//       digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
//     else
//       digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
//   }
//   else if (pressCommand == true) {
//     // FLIGHT PRESSURE
//     pressStatus = 2;

//     if (throttleValue == 0) {

//       throttleValue = initialFlightPressure;
//     }

//     if (currentPressure < throttleValue)
//       digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
//     else
//       digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
//   }
//   else {
//     // If system set to vent close pressure valve
//     digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
//     pressStatus = STATUS_OFF; // Record pressure as OFF
//   }
// }

void Pressurisation() {

  PTVoltage = analogRead(PRES_TRANS_PIN); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  Serial.print("Current pressure = ");
  Serial.print(currentPressure);
  Serial.println("");

  if (ventCommand == true) { // If vent signal is high
    digitalWrite(SOL_PIN_VENT, VALVE_OPEN); // Open the VENT solenoid
    ventStatus = STATUS_ON;
  }
  else { // If vent signal is low
    digitalWrite(SOL_PIN_VENT, VALVE_CLOSE); // CLose the VENT solenoid
    ventStatus = STATUS_OFF;
  }

  if (currentPressure < ignitionPressure)
  {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    pressStatus = STATUS_OFF;
  }
  else
  {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    pressStatus = 1;
  }
}

void thruster() {

  // If ox command is high and vent closed, but we're not blipping
  if (oxCommand == true && ventCommand == false) {
    if (oxStatus == STATUS_OFF) // If the ox is currently OFF, i.e. we just asked it to open, then record the current time as the start of flight
    {
      flightStart = millis();
    }
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    oxStatus = STATUS_ON;
  }
  else {
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    oxStatus = STATUS_OFF;
    if (!oxpulse) {
      digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    }
  }
}

void throttle()
{
  PTVoltage = analogRead(PRES_TRANS_PIN); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  throttlePressure = ((analogRead(throttlePin) / 1023 * 21 )) + 19;
  Serial.print("Throttle pressure = ");
  Serial.print(throttlePressure);
  Serial.print(" Current pressure = ");
  Serial.print(currentPressure);

  if (currentPressure < ignitionPressure )
  {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  }
  else if (throttlePressure >= currentPressure && currentPressure > ignitionPressure)
  {
    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
  }
  else if (throttlePressure < ignitionPressure && ignitionPressure < currentPressure)
  {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    Serial.print(" throttle < ig < current");
  }
  else if (throttlePressure < currentPressure && ignitionPressure < currentPressure && ignitionPressure < throttlePressure)
  {
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    Serial.print(" ig < throttle < current");

  }

  Serial.println("");
}
