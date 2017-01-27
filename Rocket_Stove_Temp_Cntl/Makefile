#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# All rights reserved
# http://embedxcode.weebly.com
#
# Last update: Sep 28, 2016 release 5.2.8


# Libraries
# ----------------------------------
# Declare application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring
# and user's libraries used 
# Short-listing libraries speeds-up building
# Typing = 0 takes none (default for application and user)
# Otherwise, leaving empty considers all (default for local)

# List application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring libraries here
# default = 0 = none
#
# TEENSY : common = AccelStepper Adafruit_CC3000 Adafruit_GFX 
#	Adafruit_ILI9340 Adafruit_ILI9341 Adafruit_NeoPixel Adafruit_nRF8001 
#	Adafruit_RA8875 Adafruit_SSD1306 Adafruit_ST7735 Adafruit_STMPE610 
#	Adafruit_VS1053 ADC AltSoftSerial Artnet Audio Bounce Bounce2 
#	CapacitiveSensor DmxSimple DogLcd DS1307RTC EasyTransfer EasyTransferI2C 
#	EEPROM Encoder Entropy Ethernet FastCRC FastLED FlexCAN 
#	FlexiTimer2 FreqCount FreqMeasure FreqMeasureMulti FrequencyTimer2 
#	i2c_t3 ILI9341_t3 IRremote Keypad ks0108 LedControl LedDisplay 
#	LiquidCrystal LiquidCrystalFast LowPower Metro MFRC522 MIDI MsTimer2 
#	NewPing NXPMotionSense OctoWS2811 OneWire openGLCD OSC Ping 
#	PS2Keyboard PulsePosition PWMServo RA8875 RadioHead 
#	ResponsiveAnalogRead SD SerialFlash Servo ShiftPWM Snooze SoftPWM 
#	SoftwareSerial SPI SPIFlash ssd1351 ST7565 ST7735_t3 Talkie 
#	TFT_ILI9163C Time TimeAlarms TimerOne TimerThree TinyGPS Tlc5940 
#	TouchScreen UTFT VirtualWire Wire x10 XBee XPT2046_Touchscreen 
#
APP_LIBS_LIST = SPI Wire

# List user's libraries here 
# default = 0 = none
#
# USER = Adafruit_MAX31855 Adafruit_Motorshield 
#	Adafruit_Motorshield_Teensy Adafruit_NeoPixel BNO055_driver CmdMessenger Cycler 
#	EightBitWaves ffft FHT_v2_0 FHT_v2_1 LED_Strip LedController MAX31855 
#	PettPlant Ping SerialCommandParser StandardCplusplus Ticker 
#
USER_LIBS_LIST = Adafruit_MotorShield SerialCommandParser TC_Interface_MAX31855


# List local libraries here
# default = empty = all
#
LOCAL_LIBS_LIST =

# List core, application and user's libraries to be excluded
# For example, WiFi may crash on Arduino 1.0.2, Esplora on Arduino 1.0.3, Firmata on Teensy 3.0,
#   OneWire on MPIDE 0023, HardwareSerial may conflict with MarlinSerial
#
#EXCLUDE_LIBS = Firmata WiFi Esplora OneWire Robot_Control Robot_Control/utility Robot_Motor


# Warning options
# ----------------------------------
# Contrary to errors, warnings don't stop compilation but they point at possible cause of errors.
# For example, unused variables with unused-variable.
#
# If 0, no warnings
# If empty, all warnings, same as WARNING_OPTIONS = all
# WARNING_OPTIONS = all no-missing-braces no-conversion-null no-pointer-arith
# WARNING_OPTIONS = unused-variable unused-function unused-label unused-value no-conversion-null no-pointer-arith
#
WARNING_OPTIONS = 0


# Applications path folder
# ----------------------------------
# Path to the folder where the IDEs are located
# By default, = /Applications
#
#APPLICATIONS_PATH = /Applications


# Serial console for Fast target
# ----------------------------------
# = 0 or = empty or comment: open serial console
# = 1: don't open serial console
#
NO_SERIAL_CONSOLE =


# Teensy USB options (default)
# ----------------------------------
# TEENSY_USB = (USB_SERIAL) USB_HID USB_SERIAL_HID USB_DISK USB_DISK_SDFLASH USB_MIDI
#   USB_RAWHID USB_FLIGHTSIM
# TEENSY_LAYOUT = (LAYOUT_US_ENGLISH) LAYOUT_CANADIAN_FRENCH LAYOUT_CANADIAN_MULTILINGUAL
#   LAYOUT_DANISH LAYOUT_FINNISH LAYOUT_FRENCH LAYOUT_FRENCH_BELGIAN LAYOUT_FRENCH_SWISS
#   LAYOUT_GERMAN LAYOUT_GERMAN_MAC
#   LAYOUT_GERMAN_SWISS LAYOUT_ICELANDIC LAYOUT_IRISH LAYOUT_ITALIAN LAYOUT_NORWEGIAN
#   LAYOUT_PORTUGUESE LAYOUT_PORTUGUESE_BRAZILIAN LAYOUT_SPANISH LAYOUT_SPANISH_LATIN_AMERICA
#   LAYOUT_SWEDISH LAYOUT_TURKISH LAYOUT_UNITED_KINGDOM LAYOUT_US_INTERNATIONAL
#
#TEENSY_USB = USB_SERIAL
#TEENSY_LAYOUT = LAYOUT_US_ENGLISH


# Teensy CPU options (default)
# ----------------------------------
# TEENSY_F_CPU = (96000000) 72000000 48000000 24000000 for Teensy 3.1
# TEENSY_F_CPU = (96000000) 48000000 24000000 for Teensy 3.0
# TEENSY_F_CPU = (48000000) 24000000 for Teensy LC
# TEENSY_F_CPU = (16000000) 8000000 4000000 for Teensy 2.0
# TEENSY_OPTIMISATION = (-Os)
#
#TEENSY_F_CPU = 48000000
#TEENSY_OPTIMISATION = -Os


# Parameters
# ----------------------------------
# Xcode takes BOARD_TAG and BOARD_PORT from the .xcconfig file
# For other IDEs than Xcode,

# BOARD_TAG is required
# If not defined, error BOARD_TAG not defined
#
#BOARD_TAG  = teensy3

# BOARD_PORT is optional
# If not defined, BOARD_PORT = /dev/tty.usb* (default)
#
#BOARD_PORT = /dev/tty.usbmodem*

# Xcode takes SKETCHBOOK_DIR from preferences.txt
# If not defined, each IDE has its own SKETCHBOOK_DIR (default)
# To share the same SKETCHBOOK_DIR along all IDEs, define
#
#SKETCHBOOK_DIR =

# SERIAL_BAUDRATE for the serial console, 9600 by default
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 9600

# OPTIMISATION for the optimisation and debugger levels
# Uncomment and specify another options
#
#OPTIMISATION    = -Os -g3


# Miscellaneous
# ----------------------------------
# Manage path with space in the name
#
CURRENT_DIR   := $(shell pwd)
CURRENT_DIR   := $(shell echo '$(CURRENT_DIR)' | sed 's/ /\\\ /g')

# C-compliant project name
#
PROJECT_NAME_AS_IDENTIFIER = Rocket_Stove_Temp_Cntl
SKETCH_EXTENSION = cpp

MAKEFILE_PATH  = $(CURRENT_DIR)/Makefiles
UTILITIES_PATH = $(CURRENT_DIR)/Utilities

include $(MAKEFILE_PATH)/Step1.mk


