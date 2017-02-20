///
/// @mainpage	Rocket_Stove_Temp_Cntl
///
/// @details	Controls a damper motor and blower to control the temperature of a rocket stove.
///
/// @author		john
/// @date		1/25/17 11:26 AM
/// @version	<#version#>
///
/// @copyright	(c) john, 2017 All rights reserved
///
/// @see		ReadMe.txt for references
///
/// @file		Rocket_Stove_Temp_Cntl.cpp
///


//#include "Arduino.h"
/*
 This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
 It won't work with v1.x motor shields! Only for the v2's with built in PWM
 control
 
 */

/*
 *  Analog pin usage from PETT_PLANT
 const uint8_t FAN_SPEED_POT_PIN = 14;        // Input ADC pin for audio data.
 pinMode(FAN_SPEED_POT_PIN, INPUT);
 samples[sampleCounter] = (float32_t)analogRead(FAN_SPEED_POT_PIN);
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SerialCommandParser.h>
#include <TC_Interface_MAX31855.h>

#define VERSION_NUMBER_MAJOR 0
#define VERSION_NUMBER_MINOR 1

// Reading of TC 1 in ice water was +1.50C
// Reading of TC 2 in ice water was +1.75C
// Reading of TC 3 in ice water was +1.50C
const float WATER_POT_TC_ZERO_OFFSET = -1.5;  // Reading of TC 1 in ice water was +1.50C

const uint8_t WATER_POT_CHIP_SELECT_PIN = 9;
const uint8_t HOME_SENSOR_PIN = 7;
const uint8_t FLUE_CHIP_SELECT_PIN = 10;

const uint8_t FAN_SPEED_POT_PIN = 14;           // Blower motor speed control potentiometer

TC_Interface_MAX31855 waterPotTcInterface;  // Create TC interface objects
TC_Interface_MAX31855 flueTcInterface;

const bool SERIAL_PARSER_DEBUG = true;
const bool SERIAL_PARSER_VERBOSE = true;

const uint8_t MOTOR_CNTLR_I2C_ADDR = 0x60;

const uint8_t DAMPER_MTR_STEPS_PER_REV = 200;
const uint8_t DAMPER_MOTOR_NUM = 1;
const uint8_t MAX_NUM_ARGS = 5;
const uint32_t SPEED_MIN = 10;
const uint32_t SPEED_MAX = 1000;
const uint32_t INITIAL_SPEED = 1000;
const uint32_t INITIAL_JOG_STEPS = 10;

const uint8_t FIRE_BLOWER_NUM = 0;
const uint8_t BLOWER_INITIAL_SPEED = 0;


enum struct ControlMode
{
  Manual    = 0,
  Automatic = 1
};

ControlMode damperControlMode;
ControlMode blowerControlMode;
uint32_t damperTempDesired;


Adafruit_MotorShield motorController(MOTOR_CNTLR_I2C_ADDR);
Adafruit_StepperMotor *damperMotor;
Adafruit_DCMotor *fireBlower;

uint32_t startTimeMillis;
uint32_t previousTcReadTime;
const uint32_t tcReadDelayMillis = 5000;

uint32_t previousDataXmitTime;
uint32_t dataXmitRateMillis = 5000;

uint32_t prevControlSystemTime;
const uint32_t CONTROL_SYSTEM_DELAY_MILLIS = 5000;
boolean isHeatOn;

uint32_t prevBlowerCntlSystemTime;
const uint32_t BLOWER_CNTL_SYS_DELAY_MILLIS = 500;

SerialCommandParser serialCommandParser;   // The demo SerialCommandParser object

int32_t fetchedArgs[MAX_NUM_ARGS];        // Used to store fetched args

// Forward function declarations
void setMoveStyle();
void moveNumSteps();
void moveNumDegrees();
void moveToPosition();
void setJogSteps();
void jogCW();
void jogCCW();
void setZeroPosition();
void setSpeed();
void runControlSystem();
void runDamperControl();
void runBlowerControl();
void checkThermocouples();
void unrecognized();
void homeDamperMotor();
void printSystemInfo();
void setdamperCntlMode();
void setblowerCntlMode();
void sendDataPacket();

void setup()
{
  Serial.begin(9600);
  delay(500);          // Allow serial port to initialize
  
//  Serial.println("Stepper test!");
  
  SPI.begin();
  
  waterPotTcInterface.setChipSelectPin(WATER_POT_CHIP_SELECT_PIN, "Water Pot");
  flueTcInterface.setChipSelectPin(FLUE_CHIP_SELECT_PIN, "Flue");

  motorController.begin();  // Begin sheild with the default frequency 1.6KHz
  
  // Setup damper stepper motor
  // Params: Steps per Rev - 200
  //         Motor number  - 1
  // Valid choices of motor number are:
  // Motor Num  Shield Terminals
  //     0      M1, M2
  //     1      M3, M4
  damperMotor = motorController.getStepper(DAMPER_MTR_STEPS_PER_REV, DAMPER_MOTOR_NUM);
  damperMotor->setSpeed(INITIAL_SPEED);
  damperMotor->setJogSteps(INITIAL_JOG_STEPS);
  damperMotor->setHomeSensor(HOME_SENSOR_PIN);

  // Setup fire blower DC motor
  fireBlower = motorController.getMotor(FIRE_BLOWER_NUM);
  fireBlower->setSpeed(BLOWER_INITIAL_SPEED);             // 0 - 255
  fireBlower->run(RELEASE);
  
  pinMode(FAN_SPEED_POT_PIN, INPUT);    // Blower motor speed control potentiometer
  
  /*
   Data packet message format
   
   a  Pot Temp Faults (0 = No faults, 1 = No connection, 2 = Short to ground, 3 = Short to VCC)
   b  Pot Temp, Actual (˚F)
   c  Damper Angle (-1 = motor not homed, 0 - 90 degrees)
   d  Damper Control Mode (0 = Manual, 1 = Auto)
   e  Motor is Homed (0 = No, 1 = Yes)
   f  Motor position (-1 = motor not homed, 0 - 199)
   g  Flue Temp Faults (0 = No faults, 1 = No connection, 2 = Short to ground, 3 = Short to VCC)
   h  Flue Temp (˚F)
   i  Fan speed, Actual (RPM)
   j  Blower Control Mode (0 = Manual, 1 = Auto)
   k  Blower Algorithm Command Speed (-1 = N/A, 0 - 100%)

     example data packet:
       '{a:<data>, b:<data>, c:<data>, d:<data>, e:<data>, f:<data>, g:<data>, h:<data>, i:<data>, j:<data>, k:<data>}'
   
   */

  //  Serial.println("Set up Commands");
  
  // Setup callbacks for SerialCommandParser commands
  serialCommandParser.addCommand("c", runControlSystem);
  serialCommandParser.addCommand("t", setMoveStyle);      // was sms
  serialCommandParser.addCommand("s", moveNumSteps);      // was mvstep
  serialCommandParser.addCommand("d", moveNumDegrees);    // was mvdeg
  serialCommandParser.addCommand("p", moveToPosition);    // was mvpos
  serialCommandParser.addCommand("h", homeDamperMotor);   //
  serialCommandParser.addCommand("j", setJogSteps);       // was setjogsteps
  serialCommandParser.addCommand("+", jogCW);             //
  serialCommandParser.addCommand("-", jogCCW);            //
  serialCommandParser.addCommand("z", setZeroPosition);   // Sets the current damper motor position as zero
  serialCommandParser.addCommand("e", setSpeed);          // was setspd
  serialCommandParser.addCommand("i", printSystemInfo);   // was setspd

//  serialCommandParser.addCommand("??", setUpdateRate);    //
  serialCommandParser.addCommand("a", setdamperCntlMode); //
  serialCommandParser.addCommand("b", setblowerCntlMode); //

  serialCommandParser.addDefaultHandler(unrecognized);    // Handler for command that isn't matched  (says "What?")
  
//  Serial.println("Ready");
  
  previousTcReadTime = millis();
  isHeatOn = false;
}

void loop()
{
  // Receive and process commands from serial port
  serialCommandParser.readSerial();     // We don't do much, just process serial commands
  
  // temperatureControlSystem.runTCS();
  
//  runControlSystem();
//  checkThermocouples();
  sendDataPacket();
}

void runControlSystem()
{
  
  runDamperControl();
  runBlowerControl();
}

void sendDataPacket()
{
  if (millis() > previousDataXmitTime + dataXmitRateMillis)
  {
    previousDataXmitTime = millis();
    
//    Serial.println("***** SendDataPacket *****");
    
    waterPotTcInterface.updateIf();
    flueTcInterface.updateIf();

    int damperTcFaults = waterPotTcInterface.getFaults();
    int flueTcFaults = flueTcInterface.getFaults();
   
    double damperTcTemp = waterPotTcInterface.getTcTempFahrenheit();
    double flueTcTemp = flueTcInterface.getTcTempFahrenheit();

    Serial.print("{a:");
    Serial.print(damperTcFaults);
    Serial.print(", b:");
    Serial.print(damperTcTemp);
    Serial.print(", g:");
    Serial.print(flueTcFaults);
    Serial.print(", h:");
    Serial.print(flueTcTemp);
    Serial.println("}");
    
  }
}

void setdamperCntlMode() {
  
}

void setblowerCntlMode() {
  
}

void checkThermocouples()
{
  if (millis() > previousTcReadTime + tcReadDelayMillis)
  {
    previousTcReadTime = millis();
    
//    Serial.println("");
//    Serial.println("** Updating TCs ******************");
    
    waterPotTcInterface.updateIf();
    flueTcInterface.updateIf();
    
//    waterPotTcInterface.displayIf();
//    flueTcInterface.displayIf();
  }
}

uint8_t fetchArgs(uint8_t numArgs)
{
  char *arg;
  uint8_t continueFetching = numArgs;
  size_t fetchedArgsIndex = 0;
  
  Serial.println("Fetching args");
  
  while (continueFetching)
  {
    arg = serialCommandParser.next();
    if (arg != NULL)
    {
      if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
      {
        Serial.print("Argument as ascii is ");
        Serial.println(arg);
      }
      
      fetchedArgs[fetchedArgsIndex] = atoi(arg);  // Converts a char string to an integer
      fetchedArgsIndex++;
      continueFetching--;
      
      if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
      {
        Serial.print("Argument ");
        Serial.print(fetchedArgsIndex);
        Serial.print(" is ");
        Serial.println(fetchedArgs[fetchedArgsIndex - 1]);
      }
    } else
    {
      // We were looking for a good arg and got null
      Serial.println("Expecting argument, got NULL");
      break;
    }
  }
  
  return !continueFetching;
}

boolean isHeatOnPos()
{
  //  if (damperMotor->getDegrees() == heatOnDegrees)
  //  {
  //    return true;
  //  }
  return isHeatOn;
}

void runBlowerControl()
{
  if (millis() > prevBlowerCntlSystemTime + BLOWER_CNTL_SYS_DELAY_MILLIS)
  {
    prevBlowerCntlSystemTime = millis();
    
    // Read pot and divide by 4
    uint32_t potVoltage = analogRead(FAN_SPEED_POT_PIN);  // Max value 1023
    uint8_t fanSpeed = potVoltage >> 2;                   // Max value 255
    
    // Add to moving average filter and calc new average
    
    // Output speed to fan
    fireBlower->setSpeed(fanSpeed);
    fireBlower->run(FORWARD);

    Serial.print("** New Fan Speed is ");
    Serial.println(fanSpeed);
  }
}

void runDamperControl()
{
  int32_t setPoint    = 75;
  int32_t setPointMin = 72;
  
  if (millis() > prevControlSystemTime + CONTROL_SYSTEM_DELAY_MILLIS)
  {
    prevControlSystemTime = millis();
    
    Serial.println("");
    Serial.println("** Updating Control System ******************");
    Serial.print("  Heat is ");
    if (isHeatOn)
    {
      Serial.println("ON");
    } else
    {
      Serial.println("OFF");
    }
    
    // Read TC
    int32_t temp = int32_t(waterPotTcInterface.getTcTempFahrenheit());
    Serial.print("  Temp (F) = ");
    Serial.println(temp);
    
    //
    if (temp < setPointMin)
    {
      if (!isHeatOnPos())
      {
        // turn on heat
        damperMotor->moveNumDegrees(90);
        isHeatOn = true;
        Serial.println("Turning on heat.");
      }
    } else
    {
      if (temp > setPoint)
      {
        if (isHeatOnPos())
        {
          // turn off heat
          damperMotor->moveNumDegrees(-90);
          isHeatOn = false;
          Serial.println("Turning off heat.");
        }
      }
      
    }
  }
}

void setMoveStyle()
{
  if (fetchArgs(1))
  {
    uint32_t moveStyle = fetchedArgs[0];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print("Move Style is ");
      Serial.println(moveStyle);
    }
    
    if ( (moveStyle >= SINGLE) && (moveStyle <= MICROSTEP) )
    {
      damperMotor->setMoveStyle(moveStyle);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing move style argument");
  }
}

void moveNumSteps()
{
  if (fetchArgs(2))
  {
    uint32_t numSteps = fetchedArgs[0];
    uint32_t dir = fetchedArgs[1];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print("numSteps is ");
      Serial.println(numSteps);
      Serial.print("dir is ");
      Serial.println(dir);
    }
    
    // TODO - replace literal ints with enums
    if (numSteps > 200)
    {
      Serial.print("Too many steps ");
      Serial.println(numSteps);
      return;
    }
    
    if ( (dir == 1) ||  (dir == 2) )
    {
      damperMotor->moveNumSteps(numSteps, dir);
    } else
    {
      Serial.print("Direction not valid: ");
      Serial.println(dir);
    }
    
  } else
  {
    Serial.println("Missing moveNumSteps argument");
  }
}

void moveNumDegrees()
{
  // Command structure:
  //   p [-]<1 - 359>
  
  if (fetchArgs(1))
  {
    int32_t degToRotate = fetchedArgs[0];
    Serial.print("Degrees to rotate is ");
    Serial.println(degToRotate);
    
    if ((degToRotate <= -1) && (degToRotate > -360) || (degToRotate >= 1) && (degToRotate < 360))
    {
      damperMotor->moveNumDegrees(degToRotate);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing moveToPosition argument");
  }
}

void moveToPosition()
{
  if (fetchArgs(1))
  {
    uint32_t newPos = fetchedArgs[0];
    Serial.print("New position is ");
    Serial.println(newPos);
    
    if ( (newPos < DAMPER_MTR_STEPS_PER_REV - 1) )
    {
      damperMotor->moveToPosition(newPos);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing moveToPosition argument");
  }
}

void setJogSteps()
{
  if (fetchArgs(1))
  {
    uint32_t numJogSteps = fetchedArgs[0];
    Serial.print("Jog Steps is ");
    Serial.println(numJogSteps);
    
    // TODO - replace literal ints with enums
    if ( (numJogSteps >= 1) && (numJogSteps <= 50) )
    {
      damperMotor->setJogSteps(numJogSteps);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing move style argument");
  }
}

void homeDamperMotor()
{
  damperMotor->homeMotor();
}

void jogCW()
{
  damperMotor->jogCW();
}

void jogCCW()
{
  damperMotor->jogCCW();
}

void setZeroPosition()
{
  damperMotor->setZeroPosition();
}

void printSystemInfo()
{
  damperMotor->printMotorInfo();
}

void setSpeed()
{
  if (fetchArgs(1))
  {
    uint32_t speed = fetchedArgs[0];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print("Speed is ");
      Serial.println(speed);
    }
    
    // TODO - replace literal ints with enums
    if ( (speed >= SPEED_MIN) && (speed <= SPEED_MAX) )
    {
      damperMotor->setSpeed(speed);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing setSpeed argument");
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
  Serial.println("Unreconized command"); 
}

