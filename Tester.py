#!/usr/bin/env python

"""
TODO:
Implement control for DACS
"""

import RPi.GPIO as GPIO
import time;
import threading;

class InputThread(object):
    def __init__(self, name, pins, pinNames):
        self.name = name;
        self.loop = True;
        self.pins = pins;
        self.pinNames = pinNames;
        self.pinPrevState = [None] * len(pins);
        self.log = [];

    def inputLoop(self):
        while (self.loop):
            for i in range(len(self.pins)):
                value = GPIO.input(self.pins[i]);
                if (value != self.pinPrevState[i]):
                    self.log.append((time.time(), self.pinNames[i] + " reads " + str(value)));
                    self.pinPrevState[i] = value;


    def breakLoop(self):
        self.loop = False;

    def getName(self):
        return self.name;

    def getLog(self):
        return self.log;

class Switcher(object):
    def indirect(self, case, args):
        method = getattr(self,case);
        return method(args);
            
    def start(self,args):
        if args[0] == "high":
            GPIO.output(START_PIN, GPIO.HIGH);
        else:
            GPIO.output(START_PIN, GPIO.LOW);
    
    def abort(self,args):
        if args[0] == "high":
            GPIO.output(ABORT_PIN, GPIO.HIGH);
        else:
            GPIO.output(ABORT_PIN, GPIO.LOW);
    
    def disarm(self,args):
        if args[0] == "high":
            GPIO.output(DISARM_PIN, GPIO.HIGH);
        else:
            GPIO.output(DISARM_PIN, GPIO.LOW);
        
    def gyro(self,args):
        gyro("x", args[0]);
        gyro("y", args[1]);
    
    def accel(self,args):
        accel("x", args[0]);
        accel("y", args[1]);
        accel("z", args[2]);
    
    def press(self,args):
        press(args[0]);
    
    def throttle(self,args):
        throttle(args[0]);
        
    def end(self,args):
        for thread in inputControllers:
            thread.breakLoop();

# Analog Outputs
PRES_TRANS_PIN = 2;  # Input from Pressure Transducer [ANALOG]
GYRO_PIN_X = 3;      # x-axis Gyro is connected to analog pin
GYRO_PIN_Y = 4;      # y-axis gyro
ACCEL_PIN_X = 14;     # x-axis accelerometer pin
ACCEL_PIN_Y = 15;     # y-axis accelerometer pin
ACCEL_PIN_Z = 17;     # z-axis accelerometer pin

# Digital Outputs
START_PIN = 18;
ABORT_PIN = 27;
DISARM_PIN = 22; 	     # New pin to be added possibly
THROTTLE_PIN = 23;

# Digital Inputs
ACT_PIN_X_POS = 24;    # ACS valve positive x_axis
ACT_PIN_X_NEG = 10;    # ACS valve negative x_axis
ACT_PIN_Y_POS = 9;    # ACS valve positive y_axis
ACT_PIN_Y_NEG = 25;  # ACS valve negative y_axis
SOL_PIN_VENT = 11;    # Output to VENT solenoid control (SV2)
SOL_PIN_PRES = 8;    # Output to PRESSURE solenoid control (SV1)
SOL_PIN_OX = 7;       # Output to OX solenoid control(SV3)

GPIO.setmode(GPIO.BCM);
GPIO.setwarnings(False);

GPIO.setup(PRES_TRANS_PIN, GPIO.OUT);
GPIO.setup(GYRO_PIN_X, GPIO.OUT);
GPIO.setup(GYRO_PIN_Y, GPIO.OUT);
GPIO.setup(ACCEL_PIN_X, GPIO.OUT);
GPIO.setup(ACCEL_PIN_Y, GPIO.OUT);
GPIO.setup(ACCEL_PIN_Z, GPIO.OUT);

GPIO.setup(START_PIN, GPIO.OUT);
GPIO.setup(ABORT_PIN, GPIO.OUT);
GPIO.setup(DISARM_PIN, GPIO.OUT);
GPIO.setup(THROTTLE_PIN, GPIO.OUT);

GPIO.setup(ACT_PIN_X_POS, GPIO.IN);
GPIO.setup(ACT_PIN_X_NEG, GPIO.IN);
GPIO.setup(ACT_PIN_Y_POS, GPIO.IN);
GPIO.setup(ACT_PIN_Y_NEG, GPIO.IN);
GPIO.setup(SOL_PIN_VENT, GPIO.IN);
GPIO.setup(SOL_PIN_PRES, GPIO.IN);
GPIO.setup(SOL_PIN_OX, GPIO.IN);

print("set pin modes");

def gyro(dim, angle):
    if (dim == "x"):
        GPIO.output(GYRO_PIN_X, 0); #ANALOG VALUE to be figured out
    else:
        GPIO.output(GYRO_PIN_Y, 0); #ANALOG VALUE to be figured out

def accel(dim, angle):
    if (dim == "x"):
        GPIO.output(ACCEL_PIN_X, 0); #ANALOG VALUE to be figured out
    elif (dim == "y"):
        GPIO.output(ACCEL_PIN_Y, 0); #ANALOG VALUE to be figured out
    else:
        GPIO.output(ACCEL_PIN_Z, 0); #ANALOG VALUE to be figured out

def press(value):
    GPIO.output(PRES_TRANS_PIN, 0);

def throttle(value):
    GPIO.output(THROTTLE_PIN, 0); #ANALOG VALUE to be figured out

def stringTimeSince(start,time,sg):
    return str(time-start)[:sg];

inputControllers = [InputThread("First",[ACT_PIN_X_POS,ACT_PIN_X_NEG,ACT_PIN_Y_POS],["ACT_PIN_X_POS", "ACT_PIN_X_NEG", "ACT_PIN_Y_POS"])];
inputControllers.append(InputThread("Second",[ACT_PIN_Y_NEG,SOL_PIN_VENT],["ACT_PIN_Y_NEG", "SOL_PIN_VENT"]));
inputControllers.append(InputThread("Third",[SOL_PIN_PRES,SOL_PIN_OX],["SOL_PIN_PRES", "SOL_PIN_OX"]));

while (True):
	fileName = input("Enter name of test file: ");
	try:
		inFile = open(fileName,"r");
		break;
	except:
		print("Error opening file");

threads = [];

try:
    for inObject in inputControllers:
        print("Creating new thread: "+ inObject.getName());
        thread = threading.Thread(target=inObject.inputLoop);
        thread.start();
        threads.append(thread);
        print("Thread: " + inObject.getName() + " created");

except Exception as e:
    print(e);
    print("Error creating thread");




log = [];
startTime = time.time();
switcher = Switcher();

# Main control loop
for line in inFile.readlines():
    cmd = line.lower().split();
    timeToSleep = float(cmd[0]) + startTime - time.time()
    if (timeToSleep>0):
        time.sleep(timeToSleep);
    log.append((time.time(), line));
    print(cmd[1]);
    switcher.indirect(cmd[1],cmd[2:]);

print("Test completed");

log.append((time.time(),"Test has concluded"));

for thread in inputControllers:
    log.extend(thread.getLog());

outputLog = sorted(log, key=lambda record: record[0]);

outFile = open(str(time.time())+"_Log.txt","w+"); # Result file

for tuple in outputLog:
    if (not "\n" in tuple[1]):

        outTuple = (tuple[0], tuple[1] + "\n");
    else:
        outTuple = tuple
    outFile.write(stringTimeSince(startTime,outTuple[0],8) + ": " + outTuple[1]);

outFile.close();

for thread in threads:
    thread.join();

GPIO.cleanup();