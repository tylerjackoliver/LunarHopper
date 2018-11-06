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

// Other
const int START_PIN = 30;
const int MECO_PIN = 2;
const int THROTTLE_PIN = A6;

// Valve constants for readability
const int VALVE_OPEN = HIGH;
const int VALVE_CLOSE = LOW;

/*
 * ACS
 */

// Gyro 
const float VCC = 5.0;                 //Gyro is running at 5V from Arduino
float xGyro0V;                         //variable to store the x -axis gyro's 0 (no rotation) voltage 
float yGyro0V;                         //variable to store the y-axis gyro's 0 (no rotation) voltage
float xVoltageSum = 0;                 //x gyro voltage         !! Potential to make locale to acsCalibration
float yVoltageSum = 0;                 //y gyro voltage         !! Potential to make locale to acsCalibration

// Accelerometer
float accelZeroVoltX = 0;              //accelerometer x axis 0 voltage
float accelZeroVoltY = 0;              //accelerometer y axis 0 voltage
float accelZeroVoltZ = 1.775;          //accelerometer z axis 0 voltage  !! Only used once and never reasinged potential to make constant and local to acs
float xAccVoltageSum = 0;              //x-axis accelerometer voltage    !! Potential to make locale to acsCalibration
float yAccVoltageSum = 0;              //y-axis acceleroometer voltage   !! Potential to make locale to acsCalibration

// ACS configuration
unsigned long timeZG = 0;                      //Variable to record the start time of zeroing the gyros
unsigned long acquisitionsCount = 0;           //Variable used to record the number of cycles used for ACS calibration !! Potential to make locale to acsCalibration
bool acsActive = false;                        //Tracking variable to detect if ACS subsystem is active
bool calibrated = false;                       //Tracking variable to record if ACS system has been calibrated
bool cycle = false;                            //Tracking variable to record if ACS system is mid cycle


const float IGNITION_PRESSURE = 21.0; // bar (abs)

/*
 * Command variables
 * Used to recored the incoming commands from the remote and PC interface
 * Initialise all to false.
 */

bool ventCommand = false;
bool actVentCommand = false;
bool mecoCommand = false;           // !!! not in use. compared against in loop and disarm, never set anywhere else, and I don't imagine it's a toggle.
bool oxCommand = false;             // !!! not in use. compared against in thruster and ACS (commented out), never set anywhere else, and I don't imagine it's a toggle.

/*
 * System status
 * Used to record the current state of the hopper
 */
bool oxStatus = false;
bool ventStatus = false;
bool mecoStatus = false;
// pressStatus does not follow the same as others,
// has an off, ignition pressure, and flight pressure
// consider changing how this behaves, or adding other
// STATUS constants for consistency across status variables
bool pressStatus = false;

double currentPressure = 0; // Current pressre of the pressure transducer

// Variables for MECO control
unsigned long mecoStart = 0;    //start time of meco
bool wasMeco = false;           //check if meco was on or off

// Variables for LOS control
unsigned long flightStart = 0;  // Time the vehicle started

// Variables for Start
bool startStatus = false; 

int blipRest = 0;
int blipFire = 0;
const int OX_NO_PULSES = 3; // Only used in loop
int oxPulseNo = 0;

float throttlePressure = 0; // !!Only used in throttle

/*
 * CODE
 */

// Setup Loop
void setup() {
  Serial.begin(9600);
  Serial.println("Starting serial process...");
  pinMode(PRES_TRANS_PIN, INPUT); //Define Pressure Transducer reading as an INPUT
  pinMode(GYRO_PIN_X, INPUT);     //reads gyro input
  pinMode(GYRO_PIN_Y, INPUT);
  pinMode(START_PIN, INPUT);
  pinMode(MECO_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ACT_PIN_X_POS, OUTPUT);
  pinMode(ACT_PIN_X_NEG, OUTPUT);
  pinMode(ACT_PIN_Y_POS, OUTPUT);
  pinMode(ACT_PIN_Y_NEG, OUTPUT);
  pinMode(SOL_PIN_VENT, OUTPUT);  //Define signal to Vent control as an OUTPUT
  pinMode(SOL_PIN_PRES, OUTPUT);  //Define signal to Pressure control as an OUTPUT
  pinMode(SOL_PIN_OX, OUTPUT);    //Define signal to OX control as an OUTPUT
}

/*
 * Main execution loop
 */

void loop() {
  Serial.print("In loop:   ");
  Serial.println(timeZG);
  if (mecoStatus == false) {

    Serial.println("MECO status: OFF");

    if (startStatus == true) {

      Serial.println("Start = 1");

      if (acsActive == false) {

        Serial.println("Performing ACS Check...");
        acsCheck();

      } else if (calibrated == false) {

        Serial.println("Performing calibration...");
        acsCalibration();

      } else if (oxPulseNo < OX_NO_PULSES) {

        Serial.print("oxPulseNo = ");
        Serial.print(oxPulseNo);
        oxPulse();
        
      } else {

        Serial.println("Working...");
        acs();
        throttle();
      }
    } else {

      startStatus = digitalRead(START_PIN);
      Serial.println("Reading start pin...");
    }

    mecoStatus = digitalRead(MECO_PIN);
    Serial.print(" MECO status = ");
    Serial.println(mecoStatus);

  } else {

    mecoHigh();
  }
}

/*
 * UPDATED TO LATEST VERSION 23/3/15 by Achal Mittal
 * Verified through system tests/simulations.
 * Will enable the systems on the hopper when activated
 */
// Never used..............
void disarm() {

  pressStatus = false;
  oxStatus = false;
  ventStatus = true;
  calibrated = false; // Reset the calibration tracking variable
  digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
  digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  digitalWrite(SOL_PIN_VENT, VALVE_OPEN);
  // !!! for the next four, need risk analysis/consideration here. 
  // !!! Going based on my interpretation of what the (conflicting) status above was saying.
  digitalWrite(ACT_PIN_X_POS, VALVE_OPEN); 
  digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);
  digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN);
  digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN);
  if (mecoCommand == true) { // If MECO has been activated
    mecoStatus = true;
  } else { // If MECO is inactive
    mecoStatus = false;
  }
}

/*
 * UPDATED TO LATEST VERSION 24/3/15 by Achal Mittal
 * Verified through system tests/simulations.
 * Will shut down the hopper systems if MECO is activated, but open the vent valve
 * Note: The MECO remote was never actually used. Neither in phase III nor in phase IV.
   The codes were therefore tested using the simulation function in the Xojo App which simulates the MECO remote being on/off and the hopper being armed/disarmed.
 */

void mecoHigh() {
  Serial.println("in MECO");
  mecoStatus = true;

  pressStatus = false;
  oxStatus = false;
  ventStatus = true;
  startStatus = false;

  // Override all the radio commands so that when MECO is turned off, the system stays shut
  blipFire = 0;
  blipRest = 0;
  actVentCommand = true;
  ventCommand = true;

  calibrated = false; //Reset calibrated variable to ensure ACS systm recalibrates when system restarts
  digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
  digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
  digitalWrite(SOL_PIN_VENT, VALVE_OPEN);
  digitalWrite(ACT_PIN_X_POS, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_Y_POS, VALVE_OPEN); // vent ACS system
  digitalWrite(ACT_PIN_Y_NEG, VALVE_OPEN); // vent ACS system

  if (!wasMeco)
  {
    mecoStart = millis();
    wasMeco = true;
  }
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
void acsCheck()
{
  if (acsActive == false) { //if ACS has just been activated, start calibration process
    calibrated = false;
    xVoltageSum = 0;
    yVoltageSum = 0;
    xAccVoltageSum = 0;   //x-axis accelerometer voltage
    yAccVoltageSum = 0;   //y-axis accelerometer voltage
    acquisitionsCount = 0;
    acsActive = true;       //indiicate ACS is active
    timeZG = millis();      //start zeroing time
    acsCalibration();      // Calibrate ACS+sensors
    Serial.print(" Checking, timeZG = ");
    Serial.println(timeZG);
  }
}

void acsCalibration() {
  const unsigned long  CALIBRATION_TIME = 40000; //Time used to zero the gyro !! Only used in acsCalibration

  if (timeZG + CALIBRATION_TIME > millis()) { // If calibration time is not over

    xVoltageSum += analogRead(GYRO_PIN_X);
    yVoltageSum += analogRead(GYRO_PIN_Y);
    xAccVoltageSum += analogRead(ACCEL_PIN_X);
    yAccVoltageSum += analogRead(ACCEL_PIN_Y);
    acquisitionsCount++; // Increment the counter
    Serial.print("Still calibrating at elapsed time: ");
    Serial.println(timeZG + CALIBRATION_TIME-millis());

  } else {
  
    if (calibrated == false) {  // If calibration time has just finished

      xGyro0V = (xVoltageSum * VCC) / (acquisitionsCount * 1023.00); // Calculate the zero rate voltage for the Z axis gyro
      yGyro0V = (yVoltageSum * VCC) / (acquisitionsCount * 1023.00); //Calculate zero rate voltage for y gro
      accelZeroVoltX = (xAccVoltageSum * VCC) / (acquisitionsCount * 1023);
      accelZeroVoltY = (yAccVoltageSum * VCC) / (acquisitionsCount * 1023);
      calibrated = true;      // Record calibration has finished
      xVoltageSum = 0;
      yVoltageSum = 0;
      xAccVoltageSum = 0;
      yAccVoltageSum = 0;   // Reset variable ready for next task
      acquisitionsCount = 0; // Reset the counter
    }
  }
}

void acs() {
  const int MAX_ANGLE = 15;                      // this angle will trigger MECO !! Only used in acs
  const unsigned long TIME_INTERVAL = 25;        //Time in milliseconds between each adjustment of the ACS system !! Only used in acs

  const float GYRO_SENSITIVITY = 0.04;   //gyro xensitivity at 40mV/deg/s !! Only used in acs
  
  const float ACCEL_SENSITIVITY = 0.8;   //accelerometer sensitivity !! Only used in acs

  const float CFF_VECTOR_GYRO = 0.90;    //complementary filter- choose what percentage of gyro angle you want in the final angle.  !! Only used in acs
  const float CFF_VECTOR_ACCEL = 0.10;   //complementary filter- choose what percentage of accelerometer angle you want in the final angle. !! Only used in acs

  const float CONTROL_LAW_GAIN = 0.5;            // gain of control law !! Only used in acs
  const float CONTROL_LAW_DEADBAND = 0.1;        // threshold for control law (deadband) #maybe this is a bit too low, might require some changes !! Only used in acs

  unsigned long timeAcs = 0;                     //Variable used to record the start time of each ACS(); loop !! Only used in acs

  if (cycle == false) { // If cycle starting for first time or has just finished

    cycle = true;       // Record the cycle is starting
    timeAcs = millis(); // Record the start time of the cycle

  } else if (timeAcs + TIME_INTERVAL < millis()) { // If time interval has passed

    cycle = false;
    int sampling_count = 0; 
    float xSum = 0.0; 
    float ySum = 0.0;
    // Get reading ten times-over sampling to reduce noise
    for (int i = 0; i < 10; i++) {
      xSum += analogRead(GYRO_PIN_X); // Add current Z gyro voltage to running total
      ySum += analogRead(GYRO_PIN_Y);
      sampling_count++; // Increment the counter
    }
    // Calculate angular rates and integrate for angles
    float xRate = ( (xSum * VCC) / (sampling_count * 1023.) - xGyro0V) / GYRO_SENSITIVITY * (1.0); // Calculate if any movement has occured in the X axis and correct angle (change sign due to how the gyro is mounted on the frame)
    float yRate = ( (ySum * VCC) / (sampling_count * 1023.) - yGyro0V) / GYRO_SENSITIVITY * (1.0); //same for y axis

    float x_g = ((analogRead(ACCEL_PIN_X) * VCC / 1023.00) - accelZeroVoltX) / ACCEL_SENSITIVITY;
    float y_g = ((analogRead(ACCEL_PIN_Y) * VCC / 1023.00) - accelZeroVoltY) / ACCEL_SENSITIVITY;
    float z_g = ((analogRead(ACCEL_PIN_Z) * VCC / 1023.00) - accelZeroVoltZ) / ACCEL_SENSITIVITY;


    float c_g = sqrt(sq(y_g) + sq(z_g));  //calculate the gravity using y and z axis
    float d_g = sqrt(sq(x_g) + sq(z_g));  // same as above using x and y axis
    float accAngleInRadiansX = -atan2 (x_g, c_g);
    float accAngleInRadiansY = -atan2 (y_g, d_g);
    float accAngleX = accAngleInRadiansX * 180 / M_PI;  //get x-angle from accelerometer in degrees
    float accAngleY = accAngleInRadiansY * 180 / M_PI;  //get y-angle from accelerometer in degrees
    float xAngle = xRate * (millis() - timeAcs + 15.00) * 0.001;  //get x angle from x-axis gyro
    float yAngle = yRate * (millis() - timeAcs + 15.00) * 0.001;  //get y angle from y-axis gyro
    float currentAngleX = CFF_VECTOR_GYRO * (xAngle) + CFF_VECTOR_ACCEL * accAngleX; //apply complementary filtering in z axis
    float currentAngleY = CFF_VECTOR_GYRO * (yAngle) + CFF_VECTOR_ACCEL * accAngleY; //apply complementary filtering in y axis


    //Apply control law and regulate the solenoid valves only after oxcommand is on
    //if (oxCommand ==true){////change
    float nextAngleX = currentAngleX + CONTROL_LAW_GAIN * xRate;
    float nextAngleY = currentAngleY + CONTROL_LAW_GAIN * yRate;

    Serial.print("ANGLE = ");
    Serial.print(abs(nextAngleX));
    if (abs(nextAngleX) < MAX_ANGLE) {

      if (abs(nextAngleX) >= CONTROL_LAW_DEADBAND) {

        if (nextAngleX > 0) {    //control law

          digitalWrite(ACT_PIN_X_POS, VALVE_OPEN);
          digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);

        } else if (nextAngleX < 0) {

          //Serial.println(currentangle+a*gyroRate);
          digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
          digitalWrite(ACT_PIN_X_NEG, VALVE_OPEN);

        } else {

          digitalWrite(ACT_PIN_X_POS, VALVE_CLOSE);
          digitalWrite(ACT_PIN_X_NEG, VALVE_CLOSE);
          
        }
      } else if (abs(nextAngleY) >= CONTROL_LAW_DEADBAND) {

        if (nextAngleY > 0) {    //control law

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

      delay(10); //Cycles was 15 before, but now is 10. if 15ms, we want 50 Hz which is 20ms. Thus, 10 gives more room to maneouvre.
      
    } else {
      mecoHigh();
    }
  }
}

void oxPulse() {
  const unsigned long BLIP_DURATION = 300; // Time in milliseconds the ox pulse lasts !! Only used in oxPulse
  const unsigned long BLIP_REST_DURATION = 5000; //duration for which the system will wait for heat to dissipare in the fuel !! Only used in oxPulse

  unsigned long timePulse = 0;    // Records start time of the last ox blip !! Only used in oxPulse

  pressurisation();

  if (pressStatus == true) {

    if (blipFire == 0 && blipRest == 0) {

      timePulse = millis();
      digitalWrite(SOL_PIN_OX, VALVE_OPEN);
      blipFire = 1;

    } else if (blipFire == 1 && blipRest == 0 && timePulse + BLIP_DURATION < millis()) {

      digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
      blipFire = 0;
      blipRest = 1;
      timePulse = millis();

    } else if (blipFire == 0 && blipRest == 1 && timePulse + BLIP_REST_DURATION < millis()) {

      blipRest = 0;
      digitalWrite(oxPulseNo, VALVE_OPEN);
      oxPulseNo++;
    }
  }
}

void pressurisation() {

  currentPressure = getPressure();

  Serial.print("Current pressure = ");
  Serial.print(currentPressure);
  Serial.println("");

  if (ventCommand == true) { // If vent signal is high

    digitalWrite(SOL_PIN_VENT, VALVE_OPEN); // Open the VENT solenoid
    ventStatus = true;

  } else { // If vent signal is low

    digitalWrite(SOL_PIN_VENT, VALVE_CLOSE); // CLose the VENT solenoid
    ventStatus = false;
  }

  if (currentPressure < IGNITION_PRESSURE) {

    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    pressStatus = false;

  } else {

    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    pressStatus = true;
  }
}

void thruster() {

  // If ox command is high and vent closed, but we're not blipping
  if (oxCommand == true && ventCommand == false) {

    if (oxStatus == false) {// If the ox is currently OFF, i.e. we just asked it to open, then record the current time as the start of flight
    
      flightStart = millis();
    }
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    oxStatus = true;
  } else {
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    oxStatus = false;
  }
}

void throttle() {
  currentPressure = getPressure();

  throttlePressure = ((analogRead(THROTTLE_PIN) / 1023 * 21 )) + 19;
  Serial.print("Throttle pressure = ");
  Serial.print(throttlePressure);
  Serial.print(" Current pressure = ");
  Serial.print(currentPressure);

  if (currentPressure < IGNITION_PRESSURE ) {

    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);

  } else if (throttlePressure >= currentPressure && currentPressure > IGNITION_PRESSURE) {

    digitalWrite(SOL_PIN_PRES, VALVE_OPEN);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);

  } else if (throttlePressure < IGNITION_PRESSURE && IGNITION_PRESSURE < currentPressure) {

    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_CLOSE);
    Serial.print(" throttle < ig < current");

  } else if (throttlePressure < currentPressure && IGNITION_PRESSURE < currentPressure && IGNITION_PRESSURE < throttlePressure) {
    
    digitalWrite(SOL_PIN_PRES, VALVE_CLOSE);
    digitalWrite(SOL_PIN_OX, VALVE_OPEN);
    Serial.print(" ig < throttle < current");

  }

  Serial.println("");
}

double getPressure(){
  int voltage = analogRead(PRES_TRANS_PIN); // Int read on analog?
  return voltage * (40 / 1023); //int * (bar/int) returns: bar
}