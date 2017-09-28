/*
 Rocket Stove Temperature Controller Firmware

 Copyright (c) john van strien, 2017 All rights reserved
 
 This project controls a damper motor and blower to control the temperature of a rocket stove.

 It is designed to run on a Teensy 3.2 and consists of the following hardware:
   - a Sparkfun Arduino shield adapter (P/N KIT-13288)
   - an Adafruit Motor Shield for Arduino v2 (P/N 1438)
   - two thermocouples
   - two Adafruit TC interface boards (P/N 269)
   - a stepper motor
   - a home sensor (for the stepper motor)
   - a 12VDC blower motor
   - a 12VDC power supply
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
const float WATER_POT_TC_ZERO_OFFSET = -1.5;

const uint8_t WATER_POT_CHIP_SELECT_PIN = 9;
const uint8_t HOME_SENSOR_PIN = 7;
const uint8_t FLUE_CHIP_SELECT_PIN = 10;

const uint8_t FAN_SPEED_POT_PIN = 14;         // Blower motor speed control potentiometer

TC_Interface_MAX31855 waterPotTcInterface;    // Create TC interface objects
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
const uint8_t BLOWER_SPEED_PERCENT_MAX = 100;

enum struct DamperControlMode : uint8_t
{
  Invalid   = 255,
  Manual    = 0,
  Automatic = 1
};

enum struct BlowerControlMode : uint8_t
{
  Invalid   = 255,
  ManualPot = 0,
  ManualUI  = 1,
  Automatic = 2
};

DamperControlMode damperControlMode;  // , prevDamperCntlMode;
BlowerControlMode blowerControlMode;  // , prevBlowerCntlMode;
//bool isNewDamperCntlMode = true;
//bool isNewBlowerCntlMode = true;

uint32_t damperTempDesired;

uint8_t blowerSpeedUI = 0;
uint8_t previousBlowerSpeedUI;
uint8_t blowerSpeedPot;
uint8_t currentBlowerSpdPercent;  // Used to store blower speed irrespective of control mode, to send to UI

Adafruit_MotorShield motorController(MOTOR_CNTLR_I2C_ADDR);
Adafruit_StepperMotor *damperMotor;
Adafruit_DCMotor *fireBlower;

uint32_t startTimeMillis;
uint32_t previousTcReadTime;
const uint32_t tcReadDelayMillis = 5000;

uint32_t previousDataXmitTime;
uint32_t dataXmitRateMillis = 5000;

uint32_t prevControlSystemTime;
const uint32_t CONTROL_SYSTEM_DELAY_MILLIS = 5500;
boolean _isHeatOn;

uint32_t prevBlowerCntlSystemTime;
const uint32_t BLOWER_CNTL_SYS_DELAY_MILLIS = 500;

int32_t potTempSetPoint = 0;
const int32_t POT_TEMP_SET_POINT_OFFSET = 3;
//int32_t potTempSetPointMin = potTempSetPoint - 3;


SerialCommandParser serialCommandParser;   // The demo SerialCommandParser object

int32_t fetchedArgs[MAX_NUM_ARGS];        // Used to store fetched args

// Forward function declarations
void setMoveStyle();
void moveNumSteps();
void moveNumDegrees();
void moveToPosition();
void moveToAngleDegrees();
void setJogSteps();
void jogCW();
void jogCCW();
void setZeroPosition();
void setDamperSpeed();
void setBlowerSpeed();
void setBlowerSpeedAsPercent(uint8_t speed);
void runControlSystem();
void runDamperControl();
void runBlowerControl();
void checkThermocouples();
void unrecognized();
void homeDamperMotor();
void printSystemInfo();
void setDamperCntlMode();
void setBlowerCntlMode();
void sendDataPacket();
void setPotTemp();
void setUpdateRate();

uint8_t fetchArgs(uint8_t numArgs);

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
   Data packet message format - Transmitted by microcontroller
   
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
   l  Blower UI speed, Set (%)
   m  Pot Temp, Desired (˚F)

     example data packet:
       '{a:<data>, b:<data>, c:<data>, d:<data>, e:<data>, f:<data>, g:<data>, h:<data>, i:<data>, j:<data>, k:<data>, l:<data>, m:<data>}'
   
   */

  //  Serial.println("Set up Commands");
  
  // Setup callbacks for SerialCommandParser commands
  serialCommandParser.addCommand("c", runControlSystem);
  serialCommandParser.addCommand("t", setMoveStyle);      // was sms
  serialCommandParser.addCommand("s", moveNumSteps);      // was mvstep
  serialCommandParser.addCommand("d", moveNumDegrees);    // was mvdeg
  serialCommandParser.addCommand("p", moveToPosition);    // was mvpos
  serialCommandParser.addCommand("n", moveToAngleDegrees);  //
  serialCommandParser.addCommand("h", homeDamperMotor);   //
  serialCommandParser.addCommand("j", setJogSteps);       // was setjogsteps
  serialCommandParser.addCommand("+", jogCW);             //
  serialCommandParser.addCommand("-", jogCCW);            //
  serialCommandParser.addCommand("z", setZeroPosition);   // Sets the current damper motor position as zero
  serialCommandParser.addCommand("e", setDamperSpeed);    //
  serialCommandParser.addCommand("f", setBlowerSpeed);    //
  serialCommandParser.addCommand("i", printSystemInfo);   //
  serialCommandParser.addCommand("q", setUpdateRate);     //
  serialCommandParser.addCommand("a", setDamperCntlMode); //
  serialCommandParser.addCommand("b", setBlowerCntlMode); //
  serialCommandParser.addCommand("m", setPotTemp);        //

  serialCommandParser.addDefaultHandler(unrecognized);    // Handler for command that isn't matched  (says "What?")
  
//  Serial.println("Ready");
  
  previousTcReadTime = millis();
  _isHeatOn = false;
  
  damperControlMode = DamperControlMode::Manual;
//  prevDamperCntlMode = DamperControlMode::Invalid;

  blowerControlMode = BlowerControlMode::ManualPot;
//  prevBlowerCntlMode = BlowerControlMode::Invalid;
  
  homeDamperMotor();
 
}

void loop()
{
  // Receive and process commands from serial port
  serialCommandParser.readSerial();

  runControlSystem();

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

    int8_t damperAngle = -1;
    uint8_t damperPos = -1;
    uint8_t isDamperHomed = damperMotor->isHomed();
    if (isDamperHomed)
    {
      damperAngle = damperMotor->getPositionDegrees();
      damperPos = damperMotor->getPositionSteps();
    }
    
    int32_t blowerCommandSpeed = -1;
    
    Serial.print("{\"a\":");
    Serial.print(damperTcFaults);

    Serial.print(", \"b\":");
    Serial.print(damperTcTemp);
    
    Serial.print(", \"c\":");
    Serial.print(damperAngle);
    
    Serial.print(", \"d\":");
    Serial.print(static_cast<uint8_t>(damperControlMode));
    
    Serial.print(", \"e\":");
    Serial.print(isDamperHomed);
    
    Serial.print(", \"f\":");
    Serial.print(damperPos);
    
    Serial.print(", \"g\":");
    Serial.print(flueTcFaults);
    
    Serial.print(", \"h\":");
    Serial.print(flueTcTemp);
    
    Serial.print(", \"i\":");
    Serial.print(currentBlowerSpdPercent);
    
    Serial.print(", \"j\":");
    Serial.print(static_cast<uint8_t>(blowerControlMode));
    
    Serial.print(", \"k\":");
    Serial.print(blowerCommandSpeed);
    
    Serial.print(", \"l\":");
    Serial.print(blowerSpeedUI);
    
    Serial.print(", \"m\":");
    Serial.print(potTempSetPoint);
    
    Serial.println("}");
    
  }
}

void setDamperCntlMode() {
  if (fetchArgs(1))
  {
    Serial.println(">>> setDamperCntlMode");

    uint8_t cntlMode = fetchedArgs[0];
//    DamperControlMode newCntlMode = DamperControlMode::Invalid;
    
    switch (cntlMode) {
      case 0:
        damperControlMode = DamperControlMode::Manual;
        break;
        
      case 1:
        damperControlMode = DamperControlMode::Automatic;
        // If the damper motor not homed, home it now.
        if (!damperMotor->isHomed())
        {
          damperMotor->homeMotor();
        }
        break;
        
      default:
        Serial.println("damperCntlMode Argument out of range");
        break;
    }
//    if ( (newCntlMode != DamperControlMode::Invalid) && (newCntlMode != prevDamperCntlMode) ) {
//      prevDamperCntlMode = damperControlMode;
//      damperControlMode = newCntlMode;
//      isNewDamperCntlMode = true;
//    }
  } else
  {
    Serial.println("Missing damperCntlMode argument");
  }
}

void setBlowerCntlMode()
{
  if (fetchArgs(1))
  {
    Serial.println(">>> setBlowerCntlMode");

    uint8_t cntlMode = fetchedArgs[0];
//    BlowerControlMode newCntlMode = BlowerControlMode::Invalid;

    switch (cntlMode) {
      case 0:
        blowerControlMode = BlowerControlMode::ManualPot;
        setBlowerSpeedAsPercent(blowerSpeedPot);
        Serial.println("** New blower control mode: manualPot");
        break;
        
      case 1:
        blowerControlMode = BlowerControlMode::ManualUI;
        // Set the blower speed to the UI speed
        Serial.print("** New blower control mode: Manual UI. Currrent UI speed is: ");
        Serial.println(blowerSpeedUI);
        setBlowerSpeedAsPercent(blowerSpeedUI);
//        previousBlowerSpeedUI = blowerSpeedUI;
        break;
        
      case 2:
        blowerControlMode = BlowerControlMode::Automatic;
        Serial.println("** New blower control mode: auto");
        break;
        
      default:
        Serial.println("** blowerControlMode Argument out of range");
        break;
    }
//    if ( (newCntlMode != BlowerControlMode::Invalid) && (newCntlMode != prevBlowerCntlMode) ) {
//      prevBlowerCntlMode = blowerControlMode;
//      blowerControlMode = newCntlMode;
//      isNewBlowerCntlMode = true;
//    }
  } else
  {
    Serial.println("Missing blowerControlMode argument");
  }
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

void setPotTemp()
{
  if (fetchArgs(1))
  {
    Serial.println(">>> setPotTemp");

    uint32_t setPoint = fetchedArgs[0];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print("New pot temp set point is ");
      Serial.println(setPoint);
    }
    
    if ( (setPoint >= 0) && (setPoint <= 250) )
    {
      potTempSetPoint = setPoint;
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing setPotTemp argument");
  }
}

boolean isHeatOn()
{
  //  if (damperMotor->getDegrees() == heatOnDegrees)
  //  {
  //    return true;
  //  }
  return _isHeatOn;
}

void runBlowerControl()
{
  if (millis() > prevBlowerCntlSystemTime + BLOWER_CNTL_SYS_DELAY_MILLIS)
  {
    prevBlowerCntlSystemTime = millis();
    
    if (blowerControlMode == BlowerControlMode::ManualPot)
    {
      uint32_t potVoltage = analogRead(FAN_SPEED_POT_PIN);  // Read pot, max value is 1023
      uint8_t fanSpeed = (potVoltage * 100) >> 10;          // Convert to percentage
      
      // Add to moving average filter and calc new average
      
      // Set speed only if integer value is different from last time
//      if ( (fanSpeed != previousBlowerSpeedPot) || (isNewBlowerCntlMode) )
      if ( fanSpeed != blowerSpeedPot )
      {
        blowerSpeedPot = fanSpeed;
//        isNewBlowerCntlMode = false;

        // Output speed to fan
        setBlowerSpeedAsPercent(fanSpeed);
        //      fireBlower->setSpeed(fanSpeed);
        //      fireBlower->run(FORWARD);
        
        Serial.print("** New Fan Speed is ");
        Serial.println(fanSpeed);
      }
    }
    else if (blowerControlMode == BlowerControlMode::ManualUI)
    {
      // Only do this the first time after a change to ManualUI control mode
//      if (blowerControlMode != prevBlowerCntlMode)
//      {
//        prevBlowerCntlMode = blowerControlMode;
//        setBlowerSpeedAsPercent(uint8_t(blowerSpeedUI));
//      }
    }
    else if (blowerControlMode == BlowerControlMode::Automatic)
    {
    }
  }
}

void runDamperControl()
{
//  int32_t setPoint    = 75;
//  int32_t setPointMin = 72;
  
  if (millis() > prevControlSystemTime + CONTROL_SYSTEM_DELAY_MILLIS)
  {
    prevControlSystemTime = millis();
    
    if (damperControlMode == DamperControlMode::Manual)
    {
    }
    else if (damperControlMode == DamperControlMode::Automatic)
    {
      Serial.println("");
      Serial.println("** Updating Damper Control System ******************");
      Serial.print("  Heat is ");
      if (_isHeatOn)
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
      
      // Check pot temp
      if (temp < (potTempSetPoint - POT_TEMP_SET_POINT_OFFSET))
      {
        if (!isHeatOn())
        {
          // Turn heat on if temp is below set point and heat is off
          damperMotor->moveToAngleDegrees(90);
          _isHeatOn = true;
          Serial.println("Turning on heat.");
        }
      } else if (temp > potTempSetPoint)
      {
        if (isHeatOn())
        {
          // Turn heat off if temp is above set point and heat is on
          damperMotor->moveToAngleDegrees(0);
          _isHeatOn = false;
          Serial.println("Turning off heat.");
        }
      }
      Serial.println();
    }
    
  }
}

void setMoveStyle()
{
  if (fetchArgs(1))
  {
    Serial.println(">>> setMoveStyle");

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
    Serial.println(">>> moveNumSteps");

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
    Serial.print(">>> Degrees to rotate is ");
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
    Serial.print(">>> New position is ");
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

void moveToAngleDegrees()
{
  if (fetchArgs(1))
  {
    uint32_t newAngle = fetchedArgs[0];
    Serial.print(">>> New angle is ");
    Serial.println(newAngle);
    
    if ((newAngle >= 0) && (newAngle < 360))
    {
      Serial.print("  moveToAngleDegrees: current position is ");
      Serial.println(damperMotor->getPositionSteps());

      Serial.println("  moveToAngleDegrees: calling moveToAngleDegrees");
      
      damperMotor->moveToAngleDegrees(newAngle);
    } else
    {
      Serial.println("Argument out of range");
    }
  } else
  {
    Serial.println("Missing moveToAngleDegrees argument");
  }
}

void setJogSteps()
{
  if (fetchArgs(1))
  {
    uint32_t numJogSteps = fetchedArgs[0];
    Serial.print(">>> Jog Steps is ");
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

void setDamperSpeed()
{
  if (fetchArgs(1))
  {
    uint32_t speed = fetchedArgs[0];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print(">>> Damper speed set to ");
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

void setBlowerSpeed()
{
  Serial.println(">>> setBlowerSpeed");

  if (fetchArgs(1))
  {
    uint32_t speed = fetchedArgs[0];
    
    Serial.print(">>>   blowerControlMode: ");
    Serial.println(static_cast<uint8_t>(blowerControlMode));

    if (blowerControlMode == BlowerControlMode::ManualUI)
    {
      if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
      {
        Serial.print("Blower speed set to ");
        Serial.print(speed);
        Serial.println("%");
      }
      
      if (speed <= BLOWER_SPEED_PERCENT_MAX)
      {
        previousBlowerSpeedUI = blowerSpeedUI;
        blowerSpeedUI = speed;
        setBlowerSpeedAsPercent(uint8_t(speed));
      } else
      {
        Serial.println("Argument out of range");
      }
    }
  } else
  {
    Serial.println("Missing setSpeed argument");
  }
}

void setBlowerSpeedAsPercent(uint8_t speed)
{
  currentBlowerSpdPercent = speed;
  fireBlower->setSpeedPercentage(speed);
  fireBlower->run(FORWARD);
}

void setUpdateRate()
{
  if (fetchArgs(1))
  {
    uint8_t rateSeconds = fetchedArgs[0];
    
    if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
    {
      Serial.print("Data update rate set to ");
      Serial.println(rateSeconds);
    }
    
    if (rateSeconds <= 30)
    {
      dataXmitRateMillis = rateSeconds * 1000;
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

uint8_t fetchArgs(uint8_t numArgs)
{
  char *arg;
  uint8_t continueFetching = numArgs;
  size_t fetchedArgsIndex = 0;
  
  if (SERIAL_PARSER_DEBUG || SERIAL_PARSER_VERBOSE)
  {
    Serial.println("Fetching args");
  }
  
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

