/*
  LICENSE

  Project Name: Steering PAD 900-F v3.2
  Author: Rui Caldas (HomeGameCoder)
  License: GNU General Public License v3.0

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <https://www.gnu.org/licenses/>.


  LEGAL DISCLAIMER

  The code provided herein is for informational and educational purposes only and is distributed
  "as is" without warranties of any kind, either express or implied, including but not limited to
  warranties of merchantability or fitness for a particular purpose.
  The author disclaims any liability for damages, losses, or other consequences arising from the
  use or misuse of this code.
  By using this code, you acknowledge and agree to these terms.
  Use entirely at your own risk.
*/

//  ███████ ███████  █████  ████████ ██    ██ ██████  ███████ ███████ 
//  ██      ██      ██   ██    ██    ██    ██ ██   ██ ██      ██      
//  █████   █████   ███████    ██    ██    ██ ██████  █████   ███████ 
//  ██      ██      ██   ██    ██    ██    ██ ██   ██ ██           ██ 
//  ██      ███████ ██   ██    ██     ██████  ██   ██ ███████ ███████ 
//                 

// Set this to true the first time you upload the script to the ProMicro.
#define isFirstTimeUploading true 
// This will ensure the data in the EEPROM is what the script is
// expecting when booting.
// It needs to be true once at the FIRST upload.
// If you leave it true, all calibrations are reseted to default
// every time you plug in the controller.
// After uploading the script once to the arduino with this true, set it to false and re-upload.
// It is not necessary to do this again!


//  ██ ███    ██  ██████ ██      ██    ██ ███████ ██  ██████  ███    ██ ███████ 
//  ██ ████   ██ ██      ██      ██    ██ ██      ██ ██    ██ ████   ██ ██      
//  ██ ██ ██  ██ ██      ██      ██    ██ ███████ ██ ██    ██ ██ ██  ██ ███████ 
//  ██ ██  ██ ██ ██      ██      ██    ██      ██ ██ ██    ██ ██  ██ ██      ██ 
//  ██ ██   ████  ██████ ███████  ██████  ███████ ██  ██████  ██   ████ ███████ 

// ADC
#include "ADS1X15.h"
ADS1115 ADS(0x48);

// DISPLAY
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

// EEPROM
#include <EEPROM.h>

// JOYSTICK
#include <Joystick.h>



Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  15,     // Buttons (Max 32)
  0,      // Hats (Max 2)
  true,   // X Axis
  true,   // Y Axis
  true,   // Z Axis
  false,  // X Rotation
  false,  // Y Rotation
  false,  // Z Rotation
  false,  // Rudder
  false,  // Throttle
  false,  // Accelerator
  false,  // Break
  false   // Steering (no force feedback here!)
);

//   ██████  ██████  ███    ██ ███████ ██  ██████  ██    ██ ██████   █████  ██████  ██      ███████ 
//  ██      ██    ██ ████   ██ ██      ██ ██       ██    ██ ██   ██ ██   ██ ██   ██ ██      ██      
//  ██      ██    ██ ██ ██  ██ █████   ██ ██   ███ ██    ██ ██████  ███████ ██████  ██      █████   
//  ██      ██    ██ ██  ██ ██ ██      ██ ██    ██ ██    ██ ██   ██ ██   ██ ██   ██ ██      ██      
//   ██████  ██████  ██   ████ ██      ██  ██████   ██████  ██   ██ ██   ██ ██████  ███████ ███████ 
//                                                                                                  
//                                                                                                  
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████                             
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██                                  
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████                             
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██                             
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████                             
//

// JOYSTICK
// Steering Wheel positions for linearity correction
int16_t steeringSensorMapLUT[11] = {
  8723,   // 0º   LEFT (+ 20º)
  9882,   // 90º
  11011,  // 180º
  12092,  // 270º
  13122,  // 360º
  14077,  // 450º CENTER
  15151,  // 540º
  16459,  // 630º
  17820,  // 720º
  19118,  // 810º
  19654   // 900º RIGHT (- 20º)
};

// STEERING LUT
const int16_t outputLUT[11] = {
  -32768, -26214, -19660, -13106, -6552,
  0, 6552, 13106, 19660, 26214, 32767
};

// Define easing modes
enum EasingMode {
  EASEIN,   // Slow start: quadratic curve (norm^2)
  LINEAR,   // Linear response: no modification
  EASEOUT   // Fast start: square root curve (sqrt(norm))
};

// Break Pedal settings (Default)
EasingMode brakeEase = EASEIN;
int16_t brakeSensorMin = 11107;
int16_t brakeSensorMax = 14793;
int16_t brakeFinalValue;

// Accelerator Pedal settings (Default)
EasingMode acceleratorEase = LINEAR;
int16_t acceleratorSensorMin = 10557;
int16_t acceleratorSensorMax = 14718;
int16_t acceleratorFinalValue;

// FORCE FEEDBACK
int forceGain = 100;
bool forceActive = true;


//   ██████  ██████  ███    ██ ███████ ████████  █████  ███    ██ ████████ 
//  ██      ██    ██ ████   ██ ██         ██    ██   ██ ████   ██    ██    
//  ██      ██    ██ ██ ██  ██ ███████    ██    ███████ ██ ██  ██    ██    
//  ██      ██    ██ ██  ██ ██      ██    ██    ██   ██ ██  ██ ██    ██    
//   ██████  ██████  ██   ████ ███████    ██    ██   ██ ██   ████    ██    
//                                                                         
//                                                                         
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████    
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██         
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████    
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██    
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████    
//


// BUTTONS
const int errorMargin = 7; // Adjustable error margin for reading the buttons

const int thresholds[7] =
{
  238, // button 3
  503, // button 5
  380, // button 6
  195, // buttons 3 & 5
  175, // buttons 3 & 6
  278, // buttons 5 & 6
  152  // buttons 3 & 5 & 6
};

const unsigned long LONG_PRESS_THRESHOLD = 1000;  // 1 second

// DISPLAY
const int DISPLAY_WIDTH = 16;

// EEPROM
const int EEPROM_START_ADDRESS = 0;

// JOYSTICK
const int16_t joystickMin = -32767;
const int16_t joystickMax = 32767;

// FORCE FEEDBACK
  Gains gains[1];
  EffectParams params[1];
  int16_t lastPosition = 0;

  #define motorPinDirection 8
  #define motorPinPWM 9
  #define motorPinEnable 10

// MENU
int menuLength = 7;



//  ██████  ██████   ██████   ██████ ███████ ███████ ███████            
//  ██   ██ ██   ██ ██    ██ ██      ██      ██      ██                 
//  ██████  ██████  ██    ██ ██      █████   ███████ ███████            
//  ██      ██   ██ ██    ██ ██      ██           ██      ██            
//  ██      ██   ██  ██████   ██████ ███████ ███████ ███████            
//                                                                      
//                                                                      
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████ 
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██      
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████ 
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██ 
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████ 
//   

// BUTTONS
int sensorValues[4];
bool buttons[15] = {false};

bool button01;
bool button02;
bool button03;
bool button04;
bool button05;
bool button06;
bool button07;
bool button08;
bool button09;
bool button10;
bool button11;
bool button12;
bool button13;
bool button14;
bool button15;
bool buttonMenu;

bool oldButton01;
bool oldButton02;
bool oldButton03;
bool oldButton04;
bool oldButton05;
bool oldButton06;
bool oldButton07;
bool oldButton08;
bool oldButton09;
bool oldButton10;
bool oldButton11;
bool oldButton12;
bool oldButton13;
bool oldButton14;
bool oldButton15;
bool oldButtonMenu;

bool button3WasPressed;
bool longPressTriggered;
long button3StartTime = 0;

bool button13WasPressed;
bool button13AsFunction;
bool button13OnHold;

// FORCE FEEDBACK
int32_t forces[1];

//LUT CONVERSION - Steering Wheel linearity Correction
float lutIn_min;
float lutIn_max;
float lutOut_min;
float lutOut_max;
float lutRatio;
int16_t mappedValue;

// MENU
int operationMode = 0;
int oldOperationMode = 0;
int menuLevel = 0;
int oldMenuLevel = 0;
int lastMenuLevel = 1;
bool reopenLevel = false;

//PEDALS
float adjusted;
int brakePedalCalibrationStep = 0;
int acceleratorPedalCalibrationStep = 0;

// STEERING
int16_t steeringSensor;
int16_t steeringPosition;
int16_t brakeSensor;
int16_t acceleratorSensor;
int steeringCalibrationStep = 0;

// SYSTEM
// Every for loop variable
int i;
int iBlock;

//  ███████ ███████ ████████ ██    ██ ██████  
//  ██      ██         ██    ██    ██ ██   ██ 
//  ███████ █████      ██    ██    ██ ██████  
//       ██ ██         ██    ██    ██ ██      
//  ███████ ███████    ██     ██████  ██      
//
void setup()
{
  Serial.begin(9600);
  #if isFirstTimeUploading
    StoreData(); //This will reset all calibrations - See FEATURES section above!
  #endif

  // SET BUTTON INPUTS
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  // START ADC ///////////////////////////////////////////////////
  ADS.setMode(0);      // 0:continuous 1:single
  ADS.setGain(1);      // 0:6.144V  1:4.096V 2:2.048V 4:1.024V 8:0.512V 16:0.256V
  ADS.setDataRate(7);  // 0:slowest 4:default 7:fastest
  ADS.begin();

  // START DISPLAY ///////////////////////////////////////////////
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  DisplayMainScreen();

  // START JOYSTICK //////////////////////////////////////////////
  Joystick.begin(true);
  Joystick.setXAxisRange(joystickMin, joystickMax);
  Joystick.setYAxisRange(joystickMin, joystickMax);
  Joystick.setZAxisRange(joystickMin, joystickMax);
  gains[0].totalGain = forceGain;
  //gains[0].constantGain      = 100;
	//gains[0].rampGain          = 0;
	//gains[0].squareGain        = 0;
	//gains[0].sineGain          = 0;
	//gains[0].triangleGain      = 0;
	//gains[0].sawtoothdownGain  = 0;
	//gains[0].sawtoothupGain    = 0;
	//gains[0].springGain        = 0;
	//gains[0].damperGain        = 100;
	//gains[0].inertiaGain       = 100;
	//gains[0].frictionGain      = 0;
	//gains[0].customGain        = 0;
  
  Joystick.setGains(gains);
  //Joystick.setEffectParams(params);

  // START FORCE FEEDBACK /////////////////////////////////////////
  // Timer3 for FFB
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 399;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TIMSK3 |= (1 << OCIE3A);
  sei();
  pinMode(motorPinDirection, OUTPUT);
  pinMode(motorPinPWM, OUTPUT);
  pinMode(motorPinEnable, OUTPUT);

  // LOAD DATA FROM EEPROM
  ReadData();
  digitalWrite(10, forceActive);
  menuLevel = 0;
  operationMode = 0;

  delay(100);
}


//  ██       ██████   ██████  ██████  
//  ██      ██    ██ ██    ██ ██   ██ 
//  ██      ██    ██ ██    ██ ██████  
//  ██      ██    ██ ██    ██ ██      
//  ███████  ██████   ██████  ██      
//
void loop() {

  // OPERATION MODES
  // 0 - In Game
  // 1 - In Menu
  // 2 - In Confirmation

  ReadButtons();
  ReadAnalogSensors();

  // IN GAME MODE
  if (operationMode == 0)  
  {
    ProcessDataAndApply();
    if (buttonMenu) {
      operationMode = 1;  // Enable MENU
      oled.clear();
      menuLevel = lastMenuLevel; // Set MENU to first setting
    }
  }

  if (operationMode == 1) MenuOperations();  // IN MENU MODE

  UpdateOldButtons();
  oldOperationMode = operationMode;

}


//  ███    ███ ███████ ███    ██ ██    ██ 
//  ████  ████ ██      ████   ██ ██    ██ 
//  ██ ████ ██ █████   ██ ██  ██ ██    ██ 
//  ██  ██  ██ ██      ██  ██ ██ ██    ██ 
//  ██      ██ ███████ ██   ████  ██████  
//
void MenuOperations()
{

  if (button03 && !oldButton03 && menuLevel != 0)  // NEXT OPTION
  {
    menuLevel += 1;
    if (menuLevel > menuLength) menuLevel = 1;
  }
  
  // GO TO SAVE CONFIRMATION
  if (button04 && !oldButton04 && menuLevel != 0) 
  {
    DisplayConfirmationScreen();
    lastMenuLevel = menuLevel;
    menuLevel = 0;

  } else if (button04 && !oldButton04 && menuLevel == 0)  // GO TO SAVE CONFIRMATION
  {
    ReadData();
    DisplayMainScreen();
    operationMode = 0;
  } else if (button05 && !oldButton05 && menuLevel == 0)  // GO TO SAVE CONFIRMATION
  {
    StoreData();
    ReadData();

    // APPLY NEW DATA RIGHT AWAY
    gains[0].totalGain = forceGain;
    Joystick.setGains(gains);

    digitalWrite(10, forceActive);

    DisplayMainScreen();
    operationMode = 0;
  }

  //    ______                              _   _           
  //   |  ____|                   /\       | | (_)          
  //   | |__ ___  _ __ ___ ___   /  \   ___| |_ ___   _____ 
  //   |  __/ _ \| '__/ __/ _ \ / /\ \ / __| __| \ \ / / _ \
  //   | | | (_) | | | (_|  __// ____ \ (__| |_| |\ V /  __/
  //   |_|  \___/|_|  \___\___/_/    \_\___|\__|_| \_/ \___|
  // 
  if (menuLevel == 1)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitleForceFeedback();
    oled.print(F("Active: "));
    if (!forceActive) { oled.print(F("No")); }
    else { oled.print(F("Yes")); }
    CleanLine();
    DisplayMenuChange();
    if (button05 && !oldButton05)  // SET NEW VALUE
    {
      forceActive = !forceActive;
      digitalWrite(10, forceActive);
    }
  }

  //    ______                  _____       _       
  //   |  ____|                / ____|     (_)      
  //   | |__ ___  _ __ ___ ___| |  __  __ _ _ _ __  
  //   |  __/ _ \| '__/ __/ _ \ | |_ |/ _` | | '_ \ 
  //   | | | (_) | | | (_|  __/ |__| | (_| | | | | |
  //   |_|  \___/|_|  \___\___|\_____|\__,_|_|_| |_|
  //
  if (menuLevel == 2)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitleForceFeedback();
    oled.print(F("Current Gain: "));
    oled.print(forceGain);
    CleanLineln();
    //oled.setRow(0); oled.setCol(0);
    oled.print(F("New value: "));
    int newGain = map(steeringSensor, steeringSensorMapLUT[0], steeringSensorMapLUT[10], 0, 200);
    if (newGain < 1) newGain = 1;
    if (newGain > 200) newGain = 200;
    oled.print(newGain);
    CleanLine();
    DisplayMenuSet();

    if (button05 && !oldButton05)  // SET NEW VALUE
    {
      forceGain = newGain;
    }
 
    DisplayMenuSet();
  }

//    ____                 _      _____         _       _ 
//   |  _ \               | |    |  __ \       | |     | |
//   | |_) |_ __ ___  __ _| | __ | |__) |__  __| | __ _| |
//   |  _ <| '__/ _ \/ _` | |/ / |  ___/ _ \/ _` |/ _` | |
//   | |_) | | |  __/ (_| |   <  | |  |  __/ (_| | (_| | |
//   |____/|_|  \___|\__,_|_|\_\ |_|   \___|\__,_|\__,_|_|
//    / ____|                                             
//   | |    _   _ _ ____   _____                          
//   | |   | | | | '__\ \ / / _ \                         
//   | |___| |_| | |   \ V /  __/                         
//    \_____\__,_|_|    \_/ \___|                         
// 
  if (menuLevel == 3)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitlePedalCurve();
    oled.println(F("Break pedal"));
    switch (brakeEase)
    {
      case EASEIN:
      oled.print("Ease In");
      break;
      case LINEAR:
      oled.print("Linear");
      break;
      case EASEOUT:
      oled.print("Ease Out");
      break;
    }
    CleanLine();

    if (button05 && !oldButton05)
    {
      brakeEase = (EasingMode)(((int)brakeEase + 1) % 3);
    }
 
    DisplayMenuChange();
  }

  //                         _                _               _____         _       _ 
  //       /\               | |              | |             |  __ \       | |     | |
  //      /  \   ___ ___ ___| | ___ _ __ __ _| |_ ___  _ __  | |__) |__  __| | __ _| |
  //     / /\ \ / __/ __/ _ \ |/ _ \ '__/ _` | __/ _ \| '__| |  ___/ _ \/ _` |/ _` | |
  //    / ____ \ (_| (_|  __/ |  __/ | | (_| | || (_) | |    | |  |  __/ (_| | (_| | |
  //   /_/____\_\___\___\___|_|\___|_|  \__,_|\__\___/|_|    |_|   \___|\__,_|\__,_|_|
  //    / ____|                                                                       
  //   | |    _   _ _ ____   _____                                                    
  //   | |   | | | | '__\ \ / / _ \                                                   
  //   | |___| |_| | |   \ V /  __/                                                   
  //    \_____\__,_|_|    \_/ \___|                                                   
  //   
  if (menuLevel == 4)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitlePedalCurve();
    oled.println(F("Accelerator pedal"));
    switch (acceleratorEase)
    {
      case EASEIN:
      oled.print("Ease In");
      break;
      case LINEAR:
      oled.print("Linear");
      break;
      case EASEOUT:
      oled.print("Ease Out");
      break;
    }
    CleanLine();

    if (button05 && !oldButton05)
    {
      acceleratorEase = (EasingMode)(((int)acceleratorEase + 1) % 3);
    }
 
    DisplayMenuChange();
  }

  //     _____      _ _ _               _   _              _____ _                 _             
  //    / ____|    | (_) |             | | (_)            / ____| |               (_)            
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __ | (___ | |_ ___  ___ _ __ _ _ __   __ _ 
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \ \___ \| __/ _ \/ _ \ '__| | '_ \ / _` |
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |____) | ||  __/  __/ |  | | | | | (_| |
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|_____/ \__\___|\___|_|  |_|_| |_|\__, |
  //                                                                                        __/ |
  //                                                                                       |___/ 

  if (menuLevel == 5)
  {
    if (menuLevel != oldMenuLevel)
    {
      steeringCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Steering wheel"));
      DisplayMenuStart();
    }

    if (button05 && button05 != oldButton05)
    {
      switch (steeringCalibrationStep)
      {
        case 0:
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Rotate full left"));
        oled.print(F("  0 deg"));
        DisplayValueText();
        break;
        
        case 1:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F(" 90 deg"));
        DisplayValueText();
        break;

        case 2:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("180 deg"));
        DisplayValueText();
        break;
        
        case 3:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("270 deg"));
        DisplayValueText();
        break;
        
        case 4:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("360 deg"));
        DisplayValueText();
        break;
        
        case 5:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Center point    "));
        oled.print(F("450 deg"));
        DisplayValueText();
        break;
        
        case 6:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        //CleanLine();
        oled.print(F("540 deg"));
        DisplayValueText();
        break;
        
        case 7:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("630 deg"));
        DisplayValueText();
        break;
        
        case 8:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("720 deg"));
        DisplayValueText();
        break;
        
        case 9:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("810 deg"));
        DisplayValueText();
        break;
        
        case 10:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("900 deg"));
        DisplayValueText();
        break;

        case 11:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep = 0;
        reopenLevel = true;
        break;
      }
    }

    if (steeringCalibrationStep > 0 && steeringCalibrationStep < 12)
    {
      oled.setRow(2); oled.setCol(94);
      oled.print(steeringSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  //     _____      _ _ _               _   _               
  //    / ____|    | (_) |             | | (_)              
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __    
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \   
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |  
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|  
  //    ____                 _      _____         _       _ 
  //   |  _ \               | |    |  __ \       | |     | |
  //   | |_) |_ __ ___  __ _| | __ | |__) |__  __| | __ _| |
  //   |  _ <| '__/ _ \/ _` | |/ / |  ___/ _ \/ _` |/ _` | |
  //   | |_) | | |  __/ (_| |   <  | |  |  __/ (_| | (_| | |
  //   |____/|_|  \___|\__,_|_|\_\ |_|   \___|\__,_|\__,_|_|
  // 
  if (menuLevel == 6)
  {
    if (menuLevel != oldMenuLevel)
    {
      brakePedalCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Break pedal"));
      DisplayMenuStart();
    }

    if (button05 && button05 != oldButton05)
    {
      switch (brakePedalCalibrationStep)
      {
        case 0:
        brakePedalCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Press the pedal 1mm"));
        oled.print(F("Minimum"));
        DisplayValueText();
        break;
        
        case 1:
        brakeSensorMin = brakeSensor;
        brakePedalCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Full pess minus 1mm"));
        oled.print(F("Maximum"));
        DisplayValueText();
        break;

        case 2:
        brakeSensorMax = brakeSensor;
        brakePedalCalibrationStep = 0;
        reopenLevel = true;
        break;
      }
    }

    if (brakePedalCalibrationStep > 0 && brakePedalCalibrationStep < 3)
    {
      oled.setRow(2); oled.setCol(85);
      oled.print(brakeSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  //     _____      _ _ _               _   _                                         
  //    / ____|    | (_) |             | | (_)                                        
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __                              
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \                             
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |                            
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|    _____         _       _ 
  //       /\               | |              | |             |  __ \       | |     | |
  //      /  \   ___ ___ ___| | ___ _ __ __ _| |_ ___  _ __  | |__) |__  __| | __ _| |
  //     / /\ \ / __/ __/ _ \ |/ _ \ '__/ _` | __/ _ \| '__| |  ___/ _ \/ _` |/ _` | |
  //    / ____ \ (_| (_|  __/ |  __/ | | (_| | || (_) | |    | |  |  __/ (_| | (_| | |
  //   /_/    \_\___\___\___|_|\___|_|  \__,_|\__\___/|_|    |_|   \___|\__,_|\__,_|_|
  // 
  if (menuLevel == 7)
  {
    if (menuLevel != oldMenuLevel)
    {
      acceleratorPedalCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Accelerator pedal"));
      DisplayMenuStart();
    }

    if (button05 && button05 != oldButton05)
    {
      switch (acceleratorPedalCalibrationStep)
      {
        case 0:
          acceleratorPedalCalibrationStep++;
          DisplayTitleCalibrate();
          oled.println(F("Press the pedal 1mm"));
          oled.print(F("Minimum"));
          DisplayValueText();
          break;
        
        case 1:
          acceleratorSensorMin = acceleratorSensor;
          acceleratorPedalCalibrationStep++;
          DisplayTitleCalibrate();
          oled.println(F("Full press minus 1mm"));
          oled.print(F("Maximum"));
          DisplayValueText();
          break;

        case 2:
          acceleratorSensorMax = acceleratorSensor;
          acceleratorPedalCalibrationStep = 0;
          reopenLevel = true;
          break;
      }
    }

    if (acceleratorPedalCalibrationStep > 0 && acceleratorPedalCalibrationStep < 3)
    {
      oled.setRow(2);
      oled.setCol(85);
      oled.print(acceleratorSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  if (reopenLevel)
  {
    reopenLevel = false;
    oldMenuLevel = -1;
  }
  else
  {
    oldMenuLevel = menuLevel;
  }

  delay(2);
}

//  ██████  ██ ███████ ██████  ██       █████  ██    ██ 
//  ██   ██ ██ ██      ██   ██ ██      ██   ██  ██  ██  
//  ██   ██ ██ ███████ ██████  ██      ███████   ████   
//  ██   ██ ██      ██ ██      ██      ██   ██    ██    
//  ██████  ██ ███████ ██      ███████ ██   ██    ██    
//

void CleanLine()
{
  oled.print(F("    "));
}
void CleanLineln()
{
  oled.println(F("    "));
}

void DisplayTitleCalibration()
{
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("CALIBRATION"));
}
void DisplayTitleCalibrate()
{
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("CALIBRATE STEERING"));
}

void DisplayTitleForceFeedback()
{
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("FORCE FEDDBACK"));
}

void DisplayTitlePedalCurve()
{
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("PEDAL LINEARITY"));
}

void DisplayRotateToAngle()
{
  oled.println(F("Rotate to angle:"));
}

void DisplayValueText()
{
  oled.print(F(" value:"));
}

void DisplayMainScreen()
{
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.print(F("STEERING PAD 900-F")); ////////////////////////////
  oled.setRow(3); oled.setCol(0);
  oled.print(F("v3.2     hold ")); /////////////////////////////
  oled.setInvertMode(true);
  oled.print(F(" menu ")); /////////////////////////////
  oled.setInvertMode(false);
  oled.print(F(">")); /////////////////////////////
}

void DisplayMenuBase()
{
  oled.setRow(3); oled.setCol(0);
  oled.setInvertMode(true);
  oled.print(F(" exit ")); ////////////////////////////
  oled.setInvertMode(false);
}
void DisplayMenuSet()
{
  DisplayMenuBase();
  oled.print(F("          ")); //////////////////
  oled.setInvertMode(true);
  oled.print(F(" set ")); /////////////////////////////
  oled.setInvertMode(false);
}
void DisplayMenuChange()
{
  DisplayMenuBase();
  oled.print(F("       ")); //////////////////
  oled.setInvertMode(true);
  oled.print(F(" change ")); /////////////////////////////
  oled.setInvertMode(false);
}

void DisplayMenuStart()
{
  DisplayMenuBase();
  oled.print(F("        ")); //////////////////
  oled.setInvertMode(true);
  oled.print(F(" start ")); /////////////////////////////
  oled.setInvertMode(false);
}

void DisplayConfirmationScreen()
{
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.print(F("Save changes?"));
  oled.setRow(3); oled.setCol(0);
  oled.setInvertMode(true);
  oled.print(F(" no ")); ////////////////////////////
  oled.setInvertMode(false);
  oled.print(F("            ")); //////////////////
  oled.setInvertMode(true);
  oled.print(F(" yes ")); /////////////////////////////
  oled.setInvertMode(false);
}


//  ██ ███    ██ ██████  ██    ██ ████████ 
//  ██ ████   ██ ██   ██ ██    ██    ██    
//  ██ ██ ██  ██ ██████  ██    ██    ██    
//  ██ ██  ██ ██ ██      ██    ██    ██    
//  ██ ██   ████ ██       ██████     ██    
//
void ReadAnalogSensors()
{
  steeringSensor = ADS.readADC(0);
  brakeSensor = ADS.readADC(1);
  acceleratorSensor = ADS.readADC(2);
}

void ReadButtons()
{
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);

  for (i = 0; i < 15; i++)
  {
    buttons[i] = false;
  }

  for (i = 0; i < 4; i++)
  {
    if (abs(sensorValues[i] - thresholds[0]) <= errorMargin) { buttons[i * 3] = true; } // Button A
    if (abs(sensorValues[i] - thresholds[1]) <= errorMargin) { buttons[i * 3 + 1] = true; } // Button B
    if (abs(sensorValues[i] - thresholds[2]) <= errorMargin) { buttons[i * 3 + 2] = true; } // Button C
    if (abs(sensorValues[i] - thresholds[3]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 1] = true; } // A + B
    if (abs(sensorValues[i] - thresholds[4]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 2] = true; } // A + C
    if (abs(sensorValues[i] - thresholds[5]) <= errorMargin) { buttons[i * 3 + 1] = true; buttons[i * 3 + 2] = true; } // B + C
    if (abs(sensorValues[i] - thresholds[6]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 1] = true; buttons[i * 3 + 2] = true; } // A + B + C
  }

  button01 = !digitalRead(4);
  button02 = !digitalRead(7);

  CheckButton3Press();
  button04 = buttons[6];
  button05 = buttons[10];
  button06 = buttons[11];
  button07 = buttons[7];
  button08 = buttons[8];
  if (!button13OnHold) button09 = buttons[3];
  button10 = buttons[4];
  if (!button13OnHold) button11 =  buttons[5];
  button12 = buttons[1];
  CheckButton13press();
  if (button13OnHold)
  {
    if(buttons[3] || buttons[5])
    {
      if(!button13AsFunction) button13AsFunction = true;
    }
  }

  if (button13AsFunction)
  {
    button14 = buttons[3];
    button15 = buttons[5];
  }
}


void UpdateOldButtons()
{
  oldButton01 = button01;
  oldButton02 = button02;
  oldButton03 = button03;
  oldButton04 = button04;
  oldButton05 = button05;
  oldButton06 = button06;
  oldButton07 = button07;
  oldButton08 = button08;
  oldButton09 = button09;
  oldButton10 = button10;
  oldButton11 = button11;
  oldButton12 = button12;
  oldButton13 = button13;
  oldButton14 = button14;
  oldButton15 = button15;
  oldButtonMenu = buttonMenu;
}


void CheckButton13press()
{
  if (button13) button13 = false; //reset button on next loop
  if (buttons[2])
  {
    if (!button13OnHold)
    {
      button13OnHold = true;
      button09 = false;
      button11 = false;
    }
  }
  else if (!buttons[2] && button13OnHold)
  {
    button13OnHold = false;
    button14 = false;
    button15 = false;
    if (!button13AsFunction) button13 = true;
    button13AsFunction = false;
  }
}


void CheckButton3Press()
{
  if (!buttons[9])
  {
    if (oldButton03) button03 = false;
    if (oldButtonMenu) buttonMenu = false;
  }

  if (buttons[9])
  {  // button pressed
    if (!button3WasPressed)
    {
      button3WasPressed = true;
      button3StartTime = millis();
      longPressTriggered = false;
    }
    else
    {
      // O botão continua pressionado: verifica se já atingiu o tempo para long press
      if (!longPressTriggered && (millis() - button3StartTime >= LONG_PRESS_THRESHOLD))
      {
        longPressTriggered = true;
        buttonMenu = true;
      }
    }
  }
  else
  {
    // O botão não está pressionado
    if (button3WasPressed)
    {
      // Detecta a transição: o botão foi solto
      if (!longPressTriggered)
      {
        button03 = true;
      }
      // Reinicia os estados para a próxima detecção
      button3WasPressed = false;
      button3StartTime = 0;
      longPressTriggered = false;
    }
  }
}





//   █████  ██    ██ ██   ██     ███████ ██    ██ ███    ██  ██████ ████████ ██  ██████  ███    ██ ███████ 
//  ██   ██ ██    ██  ██ ██      ██      ██    ██ ████   ██ ██         ██    ██ ██    ██ ████   ██ ██      
//  ███████ ██    ██   ███       █████   ██    ██ ██ ██  ██ ██         ██    ██ ██    ██ ██ ██  ██ ███████ 
//  ██   ██ ██    ██  ██ ██      ██      ██    ██ ██  ██ ██ ██         ██    ██ ██    ██ ██  ██ ██      ██ 
//  ██   ██  ██████  ██   ██     ██       ██████  ██   ████  ██████    ██    ██  ██████  ██   ████ ███████ 
//

// TIMER3 INTERRUPTION
ISR(TIMER3_COMPA_vect)
{
  Joystick.getUSBPID();  // update FFB
}


// EASE IN CALCULATION
int16_t EaseIn(int16_t sensorData, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut)
{
  float adjusted = ((float)(sensorData - minIn) / (maxIn - minIn)) * ((float)(sensorData - minIn) / (maxIn - minIn));
  return (float)minOut + adjusted * ((float)maxOut - (float)minOut);
}

// EASE OUT CALCULATION
int16_t EaseOut(int16_t sensorData, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut)
{
  float adjusted = sqrt_approx(((float)(sensorData - minIn)) / ((float)(maxIn - minIn)));
  return (float)minOut + adjusted * ((float)maxOut - (float)minOut);
}

// SQUARE ROOT FAST CALCULATION
float sqrt_approx(float x)
{
  if (x <= 0.0f)
    return 0.0f;
  float guess = x;  
  guess = 0.5f * (guess + x / guess);
  guess = 0.5f * (guess + x / guess);
  return guess;
}

// SEND DATA TO JOYSTICK
void ProcessDataAndApply()
{
  // STEERING
  steeringPosition = mapLUT(steeringSensor);

  // GET FORCE FEEDBACK
  Joystick.getForce(forces);
  //Serial.println(forces[0]);

  // Apply position
  Joystick.setXAxis(steeringPosition);



 
  //                       damping
  //                         velocity
  int16_t finalForce = forces[0] + (-(steeringPosition - lastPosition)) * 0.005; //(-velocity * (0.01 - ((float)forceGain / 200.0) * (0.01 - 0.001)));;
  
  lastPosition = steeringPosition;

  // 4. Limitar força total
  finalForce = constrain(finalForce, -255, 255);

  if (forces[0] > 0)
  {
    digitalWrite(motorPinDirection, HIGH);
    analogWrite(motorPinPWM, abs(finalForce));
  }
  else if (forces[0] < 0)
  {
    digitalWrite(motorPinDirection, LOW);
    analogWrite(motorPinPWM, abs(finalForce));
  }
  else
  {
    analogWrite(motorPinPWM, 0);
  }


  // BREAK PEDAL
  if (brakeSensor < brakeSensorMin) brakeSensor = brakeSensorMin;
  if (brakeSensor > brakeSensorMax) brakeSensor = brakeSensorMax;
  switch (brakeEase)
  {
    case EASEIN:
    brakeFinalValue = EaseIn(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;
    
    case LINEAR:
    brakeFinalValue = map(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;

    case EASEOUT:
    brakeFinalValue = EaseOut(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;
  }
  Joystick.setYAxis(brakeFinalValue);



  // ACCELERATOR PEDAL
  if (acceleratorSensor < acceleratorSensorMin) acceleratorSensor = acceleratorSensorMin;
  if (acceleratorSensor > acceleratorSensorMax) acceleratorSensor = acceleratorSensorMax;
  switch (acceleratorEase)
  {
    case EASEIN:
      acceleratorFinalValue = EaseIn(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;
      
    case LINEAR:
      acceleratorFinalValue = map(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;

    case EASEOUT:
      acceleratorFinalValue = EaseOut(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;
  }

  Joystick.setZAxis(acceleratorFinalValue);

  // BUTTONS
  if (button01 && !oldButton01) Joystick.pressButton(0);
  if (button02 && !oldButton02) Joystick.pressButton(1);
  if (button03 && !oldButton03) Joystick.pressButton(2);
  if (button04 && !oldButton04) Joystick.pressButton(3);
  if (button05 && !oldButton05) Joystick.pressButton(4);
  if (button06 && !oldButton06) Joystick.pressButton(5);
  if (button07 && !oldButton07) Joystick.pressButton(6);
  if (button08 && !oldButton08) Joystick.pressButton(7);
  if (button09 && !oldButton09) Joystick.pressButton(8);
  if (button10 && !oldButton10) Joystick.pressButton(9);
  if (button11 && !oldButton11) Joystick.pressButton(10);
  if (button12 && !oldButton12) Joystick.pressButton(11);
  if (button13 && !oldButton13) Joystick.pressButton(12);
  if (button14 && !oldButton14) Joystick.pressButton(13);
  if (button15 && !oldButton15) Joystick.pressButton(14);


  if (!button01 && oldButton01) Joystick.releaseButton(0);
  if (!button02 && oldButton02) Joystick.releaseButton(1);
  if (!button03 && oldButton03) Joystick.releaseButton(2);
  if (!button04 && oldButton04) Joystick.releaseButton(3);
  if (!button05 && oldButton05) Joystick.releaseButton(4);
  if (!button06 && oldButton06) Joystick.releaseButton(5);
  if (!button07 && oldButton07) Joystick.releaseButton(6);
  if (!button08 && oldButton08) Joystick.releaseButton(7);
  if (!button09 && oldButton09) Joystick.releaseButton(8);
  if (!button10 && oldButton10) Joystick.releaseButton(9);
  if (!button11 && oldButton11) Joystick.releaseButton(10);
  if (!button12 && oldButton12) Joystick.releaseButton(11);
  if (!button13 && oldButton13) Joystick.releaseButton(12);
  if (!button14 && oldButton14) Joystick.releaseButton(13);
  if (!button15 && oldButton15) Joystick.releaseButton(14);

}

// RETURN LUT CALCULATED VALUE

int16_t mapLUT(int16_t inputValue)
{
  // Out of limits
  if (inputValue <= steeringSensorMapLUT[0]) return outputLUT[0];
  if (inputValue >= steeringSensorMapLUT[10]) return outputLUT[10];

  for (iBlock = 0; iBlock < 10; iBlock++) {
    int16_t in_min = steeringSensorMapLUT[iBlock];
    int16_t in_max = steeringSensorMapLUT[iBlock + 1];

    if ((iBlock < 9 && inputValue >= in_min && inputValue < in_max) ||
        (iBlock == 9 && inputValue >= in_min && inputValue <= in_max)) {

      // Offset input to 0
      int32_t in_rel = inputValue - in_min;
      int32_t in_range = in_max - in_min;

      // Offset output to 0
      int32_t out_range = outputLUT[iBlock + 1] - outputLUT[iBlock];

      // Interpolação relativa com arredondamento
      int32_t rel_output = (in_rel * out_range + (in_range / 2)) / in_range;

      // Somar mínimo do bloco para obter output absoluto
      return (int16_t)(outputLUT[iBlock] + rel_output);
    }
  }
}


//  ███████ ███████ ██████  ██████   ██████  ███    ███ 
//  ██      ██      ██   ██ ██   ██ ██    ██ ████  ████ 
//  █████   █████   ██████  ██████  ██    ██ ██ ████ ██ 
//  ██      ██      ██      ██   ██ ██    ██ ██  ██  ██ 
//  ███████ ███████ ██      ██   ██  ██████  ██      ██ 
// 

void StoreData()
{
  int address = EEPROM_START_ADDRESS;

  // ADDRESS BOOK
  // 0 (22 bytes) Steering LUT Array
  // 21
  //////////////////////////////////
  // 22 (4 bytes) Brake Pedal
  // 25
  //////////////////////////////////
  // 26 (4 bytes) Accelerator Pedal
  // 29
  //////////////////////////////////
  // 30 (2 bytes) Force Gain
  // 31
  //////////////////////////////////
  // 32 (1 byte) Force Active
  //////////////////////////////////
  // 33 (1 byte) Brake Easing Mode
  //////////////////////////////////
  // 34 (1 byte) Accelerator Easing Mode
  //////////////////////////////////

  // Steering LUT Array
  for (i = 0; i < 11; i++)
  {
    EEPROM.put(i * sizeof(int16_t), steeringSensorMapLUT[i]);
  }
  
  // Brake Pedal
  EEPROM.put(22, brakeSensorMin);
  EEPROM.put(24, brakeSensorMax);
  
  // Accelerator Pedal
  EEPROM.put(26, acceleratorSensorMin);
  EEPROM.put(28, acceleratorSensorMax);
  
  // Force Feedback Gain
  EEPROM.put(30, forceGain);
  
  // Force Feedback Active
  EEPROM.put(32, forceActive);

  // Store Easing Modes as one byte each
  EEPROM.put(33, (uint8_t) brakeEase);
  EEPROM.put(34, (uint8_t) acceleratorEase);
}

void ReadData()
{

  // Steering LUT Array
  for (i = 0; i < 11; i++)
  {
    EEPROM.get(i * sizeof(int16_t), steeringSensorMapLUT[i]);
    //Serial.println(steeringSensorMapLUT[i]);
  }


  // Break Pedal
  EEPROM.get(22, brakeSensorMin);
  EEPROM.get(24, brakeSensorMax);

  
  // Accelerator Pedal
  EEPROM.get(26, acceleratorSensorMin);
  EEPROM.get(28, acceleratorSensorMax);

  // Force Feedback Gain
  EEPROM.get(30, forceGain);
  
  // Force Feedback Active
  EEPROM.get(32, forceActive);

  // Read the Easing Modes
  uint8_t temp;
  EEPROM.get(33, temp);
  brakeEase = static_cast<EasingMode>(temp);
  
  EEPROM.get(34, temp);
  acceleratorEase = static_cast<EasingMode>(temp);
  
}