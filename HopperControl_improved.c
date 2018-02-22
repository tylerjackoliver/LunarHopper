// Make their code betterer - Quoted by Jack Tyler

// Initialise Variables
ACS_Already_Callibrated = 0
ACSActive = False
Calibrated = False

oxPulseNo = 0 // Initial pulse no
oxNoPulses = 3 // Blip engine 3 times

void setup() {  

  ACS_check(); // Has ACS_Callibration in function

  }



void loop() {

  while (armCommand == false){ // Controller off, System DISARMED

   disarm();

   if (mecoCommand == 1){
      mecoStatus = 1
      meco_high();
     }

   ACT_Test(); // Check ACT, requires command input from controller

  }

  while (armCommand == True){

    if (mecoCommand == 1){
      mecoStatus = 1
      meco_high();
    }

    while (oxPulseNo < oxNoPulses){
      Pressurisation();
      ox_Pulse();
      oxPulseNo++;
    }

    Pressursation(); // Pressurse tanks
    // FUNCITON FOR CHECKING PRESSURE IN TANKS
    Throttle();


    if (mecoCommand == 1){
      mecooStatus = 1
      meco_high();
    }

    ACS(); // ACS control

    }
  }


void meco_high() {
  // This function turns on MECO
  // Edited

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


void ACS_check(){
  // This function callibrates ACS

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
  ACS(); // Start main ACS function after callibration is complete

  }


void ACS_Calibration() {
  // Function callibrates ACS
  // Edited

  while (TimeZG + CalibrationTime > millis()) { //Callibrate for 40 seconds
    x_voltage_sum += analogRead(gyroPin_x);
    y_voltage_sum += analogRead(gyroPin_y);
    xacc_voltage_sum += analogRead(xpin);
    yacc_voltage_sum += analogRead(ypin);
    acquisitions_count++; // Increment the counter
    Serial.print("still calibrating at ");
    Serial.println(TimeZG + CalibrationTime-millis());
  }

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


void ACT_Test(){
  //ACTtestcommand used before flight to check the ACS valves are working properly
  //The command will open two valves on each axis for 15 ms one after the other

    if (ACTtestcommand == true) { //if ACTtestcommand signal is high

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

      ACTtestcommand = false;

    } else if (ACTventcommand == true) { //when ACTventcommand signal is high

      //set ACTventstatus to on and open all valves
      ACTventstatus = 1; 
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin3, HIGH);
      digitalWrite(ledPin4, HIGH);

      // Shut all valves
      ACSstatus = 0;
      ACTventstatus = 0;
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin3, LOW);
      digitalWrite(ledPin4, LOW);

      ACSActive = false; // ACS Needs recallibrating
      ACS_Callibration(); // Recallibrate ACS
      }
    }


void ACS() {
  // This function controls opening and closing of Valve
  // PID Should be implemented here
  // Not edited

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


void throttle() {

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


void Pressurisation() {
  // Presssurises oxi tanks

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

