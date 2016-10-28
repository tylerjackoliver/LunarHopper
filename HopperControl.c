/*
 * PIN NUMBERS
 * Set pins for various input and output devices on arduino. e.g.- gyro, accelerometer,pressure transducer and solenoid valves.
 */

const int PT = A4; //Input from Pressure Transducer [ANALOG]
int gyroPin_x = A0; //x-axis Gyro is connected to analog pin
int gyroPin_y = A5; //y-axis gyro
int xpin = A1;
int ypin = A2;
int zpin = A3;
int ledPin1 = 6;//ACS valve right x_axis
int ledPin2 = 5;//ACS valve left x_axis
int ledPin3 = 3;//ACS valve left y_axis
int ledPin4 = 4;//ACS valve right y_axis
const int vent = 12; //Output to VENT solenoid control (SV2)
const int pressure = 10; //Output to PRESSURE solenoid control (SV1)
const int ox = 11; //Output to OX solenoid control(SV3)


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
boolean Cycle = false; //Tracking variable to record if ACS system is mid cycle
boolean wasOXon = false; //Variable to track if control law can be activated based on whether OX caommand has been given or not.
unsigned long sampling_count = 0;//used within ACS function in conjunction with z_sum and y_sum. records the no. of readings taken to reduce noise.
unsigned long TimeZG = 0; //Variable to record the start time of zeroing the gyros
boolean ACSActive = false; //Tracking variable to detect if ACS subsystem is active
boolean Calibrated = false; //Tracking variable to record if ACS system has been calibrated
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
const float descentPressure = 24.5; // bar abs. This is the pressure demanded by the LOS autopilot control. Should be pressure for hover at dry hopper - 4 bar
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

//boolean armCommand = false;
boolean armCommand = false;
boolean ACScommand = false;
boolean ACTtestcommand = false;
boolean ACTventcommand = false;
boolean pressCommand = false;
boolean pressIgnitionCommand = false;
boolean oxCommand = false;
boolean oxBlipCommand = false;
boolean ventCommand = false;
boolean mecoCommand = false;
double throttleValue = 0.0;

/*
 * System status
 * Used to record the current state of the hopper
 */

int armStatus = 0;
int ACSstatus = 0;
int ACTteststatus = 0;
int ACTventstatus = 0;
int pressStatus = 0;
int oxStatus = 0;
int ventStatus = 0;
int mecoStatus = 0;

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
boolean oxpulse = false; //Tracking variable used to record if oxpulse has been started

// Variables for MECO control
unsigned long mecoStart = 0;//start time of meco
boolean wasMECO = false;//check if meco was on or off

// Variables for LOS control
unsigned long flightStart = 0;//variable to store flight time

// Variables for Start
int startStatus = 0;
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
  Serial.println("serial start");
  pinMode(PT, INPUT); //Define Pressure Transducer reading as an INPUT
  pinMode(gyroPin_x, INPUT);//reads gyro input
  pinMode(gyroPin_y, INPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(vent, OUTPUT); //Define signal to Vent control as an OUTPUT
  pinMode(pressure, OUTPUT); //Define signal to Pressure control as an OUTPUT
  pinMode(ox, OUTPUT); //Define signal to OX control as an OUTPUT
  pinMode(startPin, INPUT);
  pinMode(mecoPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(26, OUTPUT);
  Serial.println("before high");
  digitalWrite(24, HIGH);
  Serial.println("after high");
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
      armStatus = 1; // Record armStatus as ON

      // Don't listen if MECO has just been turned on
      if (mecoStart + mecoMinHold < millis())
      {
        if (mecoCommand == true) {
          // MECO is activated
          mecoStatus = 1;
          meco_high();
        } else {
          // NORMAL OPERATION (MECO DOWN, ARM ON)
          wasMECO = false;
          mecoStatus = 0;
          ACS_check(); // Run ACS subsystem function
          pressurization(); // Run Vent/Pressure subsystem function
          ox_pulse(); // Run OX PULSE function
          thruster(); // Run THRUSTER function
          }
      }
    } */
  Serial.print("in loop   ");
  Serial.println(TimeZG);
  if (mecoStatus == 0)
  {
    Serial.println("meco = 0");
    if (startStatus == 1)
    {
      Serial.println("start = 1");
      digitalWrite(24, LOW);
      if (ACSActive == false)
      {
        Serial.println("acs check");
        ACS_Check();
      }
      else if (Calibrated == false)
      {
        Serial.println("calibrate");
        ACS_Calibration();
      }
      else if (oxPulseNo < oxNoPulses)
      {
        Serial.print("oxpulseno = ");
        Serial.print(oxPulseNo);
        ox_Pulse();
      }
      else
      {
        Serial.println("stuff in working");
        ACS();
        throttle();
      }
    }

    else
    {
      startStatus = digitalRead(startPin);
      Serial.println("read start pin");
    }
    mecoStatus = digitalRead(mecoPin);
    Serial.print(" meco status = ");
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
  boolean wasDisarmed = (armStatus == 1) ? true : false; // Record to be true if the hopper has just been disarmed (i.e. its status is still set to armed)
  armStatus = 0; // Record armStatus as OFF
  pressStatus = 0; // Record pressStatus as OFF
  oxStatus = 0; // Record oxStatus as OFF
  ACSstatus = 0; // Record acsStatus as OFF
  ACTventstatus = 0; //Recored ACTventstatus as ON
  ventStatus = 0; // Record ventStatus as ON
  Calibrated = false; // Reset the calibration tracking variable
  PTVoltage = analogRead(PT); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage * 39) / 1023) + 1; // Calculate pressure in system in bar (abs)
  digitalWrite(pressure, LOW); // Close the PRESSURE solenoid
  digitalWrite(ox, LOW); // Close the OX solenoid
  digitalWrite(vent, LOW); // Open the VENT solenoid
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
  if (mecoCommand == true) { // If MECO has been activated
    mecoStatus = 1; // Record mecoStatus as ON
  } else { // If MECO is inactive
    mecoStatus = 0; // Record mecoStatus as OFF
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
  digitalWrite(28, HIGH);
  digitalWrite(24, LOW);
  mecoStatus = 1;

  pressStatus = 0; // Record Pressure Status as OFF
  oxStatus = 0; // Record Ox Status as OFF
  ACSstatus = 0; // Record ACS Status as OFF
  ACTventstatus = 1; //Record ACTventstatus as ON
  ventStatus = 1; // Record vent status as ON
  startStatus = 0;

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
  PTVoltage = analogRead(PT); //Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage * 39) / 1023) + 1; //Calculate pressure in system in bar (abs)
  digitalWrite(ox, LOW); // Close the ox solenoid
  digitalWrite(pressure, LOW); // Close the PRESSURE solenoid
  digitalWrite(vent, HIGH); // Open the VENT solenoid
  //Open ACT valves to vent ACS system:
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH);

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
    ACS_Calibration();
    Serial.print(" in check, Timezg = ");
    Serial.println(TimeZG);
  }
}

void ACS_check() {
  if (ACScommand == true) { //If ACS signal is high
    ACSstatus = 1;//set ACSstatus to 1

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
    ACSstatus = 2; //set acs status to testing
    for (i = 0; i < 1; i++) { //run just once
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, HIGH);
      delay(15);
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin3, HIGH);
      digitalWrite(ledPin4, HIGH);
      delay(15);
      digitalWrite(ledPin3, LOW);
      digitalWrite(ledPin4, LOW);
    }
    ACTtestcommand = false;
  }
  else if (ACTventcommand == true) { //when ACTventcommand signal is high
    ACTventstatus = 1; //set ACTventstatus to on and open all valves
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
  }

  else {
    ACSstatus = 0;//otherwise set ACSstatus and ACTventstatus to off and close all solenoid valves
    ACTventstatus = 0;
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    ACSActive = false;//restart calibration process in next run

  }

}

void ACS_Calibration() {
  if (TimeZG + CalibrationTime > millis()) { // If calibration time is not over
    x_voltage_sum += analogRead(gyroPin_x);
    y_voltage_sum += analogRead(gyroPin_y);
    xacc_voltage_sum += analogRead(xpin);
    yacc_voltage_sum += analogRead(ypin);
    acquisitions_count++; // Increment the counter
    Serial.print("still calibrating at ");
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
      y_sum += analogRead(gyroPin_y);
      x_sum += analogRead(gyroPin_x); // Add current Z gyro voltage to running total
      sampling_count++; // Increment the counter
    }
    // Calculate angular rates and integrate for angles
    xRate = ( (x_sum * Vcc) / (sampling_count * 1023.) - XGyro0V) / gyroSensitivity * (1.0); // Calculate if any movement has occured in the X axis and correct angle (change sign due to how the gyro is mounted on the frame)
    yRate = ( (y_sum * Vcc) / (sampling_count * 1023.) - YGyro0V) / gyroSensitivity * (1.0); //same for y axis

    float x_g = ((analogRead(xpin) * Vcc / 1023.00) - x_accZeroVoltage) / accSensitivity;
    float y_g = ((analogRead(ypin) * Vcc / 1023.00) - y_accZeroVoltage) / accSensitivity;
    float z_g = ((analogRead(zpin) * Vcc / 1023.00) - z_accZeroVoltage) / accSensitivity;


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
          digitalWrite(ledPin1, HIGH);            // open nozzle  LED1 on
          digitalWrite(ledPin2, LOW);             //close nozzle  LED2 off


        }
        else if (currentangle_x + a * xRate < 0) {
          //Serial.println(currentangle+a*gyroRate);
          digitalWrite(ledPin2, HIGH);            // open nozzle  LED2 on
          digitalWrite(ledPin1, LOW);             //close nozzle  LED1 off

        }
        else {
          digitalWrite(ledPin2, LOW);            // open nozzle  LED2 off
          digitalWrite(ledPin1, LOW);             //close nozzle  LED1 off
        }
      }
      else if (abs(currentangle_y + a * yRate) >= k) {
        if (currentangle_y + a * yRate > 0) {    //control law
          digitalWrite(ledPin3, HIGH);            // open nozzle  LED3 on
          digitalWrite(ledPin4, LOW);             //close nozzle  LED4 off

        }
        else if (currentangle_y + a * yRate < 0) {

          digitalWrite(ledPin4, HIGH);            // open nozzle  LED4 on
          digitalWrite(ledPin3, LOW);             //close nozzle  LED3 off

        }
        else {
          digitalWrite(ledPin3, LOW);            // open nozzle  LED3 off
          digitalWrite(ledPin4, LOW);             //close nozzle  LED4 off
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
void ox_pulse() {
  if (oxBlipCommand == true && pressStatus == 1) {
    // If Blip command is active and system is pressurised for catalyst warm up
    if (oxpulse == false) { // If oxpulse command received this loop
      Timepulse = millis(); // Record start time of pulse
      oxpulse = true; // Record the oxpulse command has been received
      digitalWrite(ox, HIGH); // Open the OX solenoid
    } else {
      if (Timepulse + blipDuration > millis()) { //If blip duration has passed
        digitalWrite(ox, HIGH); // Open the OX solenoid
      } else {
        digitalWrite(ox, LOW); // Close the OX solenoid
        oxpulse = false; // Reset the tracking variable
        oxBlipCommand = false;  // Cancel signal to enable blip effect
      }
    }
  }
}
void ox_Pulse() {
  PTVoltage = analogRead(PT); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  Pressurisation();

  if (pressStatus == 1) {
    if (blipFire == 0 && blipRest == 0)
    {
      Timepulse = millis();
      digitalWrite(ox, HIGH);
      blipFire = 1;
      digitalWrite(26, HIGH);
    }
    else if (blipFire == 1 && blipRest == 0 && Timepulse + blipDuration < millis())
    {
      digitalWrite(ox, LOW);
      digitalWrite(26, LOW);
      blipFire = 0;
      blipRest = 1;
      Timepulse = millis();
    }
    else if (blipFire == 0 && blipRest == 1 && Timepulse + blipRestDuration < millis())
    {
      blipRest = 0;
      digitalWrite(7 + oxPulseNo, HIGH);
      oxPulseNo++;
    }
  }
}

/*
 * Modified and updated to latest version on 24/3/15 by Achal Mittal
 * Verified to work in May 2015 by using LEDs and potentiometer(to simulate pressure transducer) and throttle.
 * Controls the flow of ox to the rocket during flight
 *
 */
void pressurization() {

  PTVoltage = analogRead(PT); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 8) + 1)  ; // Calculate pressure in system in bar (abs)

  if (ventCommand == true) { // If vent signal is high
    digitalWrite(vent, HIGH); // Open the VENT solenoid
    ventStatus = 1; // Record ventStatus as active
  }
  else { // If vent signal is low
    digitalWrite(vent, LOW); // CLose the VENT solenoid
    ventStatus = 0; //Record ventStatus as closed
  }

  if (pressIgnitionCommand == true) {
    // IGNITION PRESSURE
    // System set to lower pressure to warm up the catalyst bed
    pressStatus = 1; // Record system set to deliver the ignition pressure
    digitalWrite(pressure, (currentPressure < ignitionPressure) ? HIGH : LOW); //this is an integrated if statement. If currentPressure < ignitionPressure, the system will write HIGH, otherwise it will write LOW
  }
  else if (pressCommand == true) {
    // FLIGHT PRESSURE
    pressStatus = 2;

    if (throttleValue == 0) {

      throttleValue = initialFlightPressure;
    }

    digitalWrite(pressure, (currentPressure < throttleValue) ? HIGH : LOW);
  }
  else {
    // If system set to vent close pressure valve
    digitalWrite(pressure, LOW);
    pressStatus = 0; // Record pressure as OFF
  }
}

void Pressurisation() {

  PTVoltage = analogRead(PT); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  Serial.print("current Pressure = ");
  Serial.print(currentPressure);
  Serial.println("");
  
  if (ventCommand == true) { // If vent signal is high
    digitalWrite(vent, HIGH); // Open the VENT solenoid
    ventStatus = 1; // Record ventStatus as active
  }
  else { // If vent signal is low
    digitalWrite(vent, LOW); // CLose the VENT solenoid
    ventStatus = 0; //Record ventStatus as closed
  }

  if (currentPressure < ignitionPressure)
  {
    digitalWrite(pressure, HIGH);
    pressStatus = 0;
  }
  else
  {
    digitalWrite(pressure, LOW);
    pressStatus = 1;
  }
}

void thruster() {

  // If ox command is high and vent closed, but we're not blipping
  if (oxCommand == true && ventCommand == false) {
    if (oxStatus == 0) // If the ox is currently OFF, i.e. we just asked it to open, then record the current time as the start of flight
    {
      flightStart = millis();
    }
    digitalWrite(ox, HIGH); // Open the ox solenoid
    oxStatus = 1; // Record ox as ON
  }
  else {
    digitalWrite(ox, LOW); // Close the ox solenoid
    oxStatus = 0;
    if (!oxpulse) {
      digitalWrite(ox, LOW);
    }
  }
}

void throttle()
{
  PTVoltage = analogRead(PT); // Read the voltage from the pressure transducer
  currentPressure = ((PTVoltage / 1023 * 39) + 1)  ; // Calculate pressure in system in bar (abs)

  throttlePressure = ((analogRead(throttlePin) / 1023 * 21 )) + 19;
  Serial.print("throttle pressure = ");
  Serial.print(throttlePressure);
  Serial.print(" current pressure = ");
  Serial.print(currentPressure);
  
  if (currentPressure < ignitionPressure )
  {
    digitalWrite(pressure, HIGH);
    digitalWrite(ox, LOW);
    digitalWrite(26, LOW);
  }
  else if (throttlePressure >= currentPressure && currentPressure > ignitionPressure)
  {
    digitalWrite(pressure, HIGH);
    digitalWrite(ox, HIGH);
    digitalWrite(26, HIGH);
  }
  else if (throttlePressure < ignitionPressure && ignitionPressure < currentPressure)
  {
    digitalWrite(pressure, LOW);
    digitalWrite(ox, LOW);
    digitalWrite(26, LOW);
    Serial.print(" throttle < ig< current");
  }
  else if (throttlePressure < currentPressure && ignitionPressure < currentPressure && ignitionPressure < throttlePressure)
  {
    digitalWrite(pressure, LOW);
    digitalWrite(ox, HIGH);
    digitalWrite(26, HIGH);
    Serial.print(" ig < throttle < current");
    
  }

  Serial.println("");
}

