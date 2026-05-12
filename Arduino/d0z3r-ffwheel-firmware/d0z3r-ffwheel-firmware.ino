// ADC
const uint8_t SCL_PIN = SCL;
const uint8_t SDA_PIN = SDA;
const uint8_t DELAY_MICROS = 0;

#include <AceWire.h>
#include <DigitalWriteFast.h>
#include <ace_wire/SimpleWireFastInterface.h>
using ace_wire::SimpleWireFastInterface;

using WireInterface = SimpleWireFastInterface<SDA_PIN, SCL_PIN, DELAY_MICROS>;
WireInterface wireInterface;
#include "src/usb_rename.h"

#include "ADS1X15.h"
ADS1115<WireInterface> ADS(0x48, &wireInterface);

#include "SSD1306Ascii.h"
#include "SSD1306AsciiAceWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAceWire<WireInterface> oled(wireInterface);

#include <EEPROM.h>
#include <Joystick.h>

#define isFirstTimeUploading false

// Alt mode detection (hold button01 or button02 during boot for T300RS mode)
bool altMode = false;
struct EarlyInit {
    EarlyInit() {
        pinMode(14, INPUT_PULLUP);
        pinMode(16, INPUT_PULLUP);
        if(!digitalRead(14) || !digitalRead(16)){
          altMode = true;
        }
    }
} earlyInit; 
  

#define ENCODER_A 0
#define ENCODER_B 1
#define PULSES_PER_REV 840  

#define EN_PIN   10
#define PWM_A    9
#define PWM_B   8

float PWM_MAX = 250;

float Kc   = 0.0f;   // NO local centering - pure direct drive mode
float Bd   = 0.0f;   // Start with NO damping - add only if needed
float Gain = 1.2f;   // Higher gain now that we removed PWM minimum
float deadzone = 15.0f;  // Higher deadzone to prevent drift
int8_t forceInvert = 1;  // 1 = normal, -1 = inverted

bool enableGameFFB = true;  // Game FFB enabled

// Center calibration - initialize to 0 to prevent garbage values
long encoderOffset = 0;  // Offset to calibrate center position

int16_t brakeSensorMin = 10000;  // Adjusted based on logs
int16_t brakeSensorMax = 23000;  // Adjusted based on logs
int16_t brakeFinalValue;

int16_t acceleratorSensorMin = 9400;  // Fixed based on your logs (9472 min observed)
int16_t acceleratorSensorMax = 10600; // Fixed based on your logs (10506 max observed)
int16_t acceleratorFinalValue;

int brakePedalCalibrationStep = 0;
int acceleratorPedalCalibrationStep = 0;


float steeringSensor = 0;

Gains gains[1];
EffectParams params[1];
int32_t forces[1] = {0};  // Initialize to zero to prevent garbage values

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  15,     // Buttons
  0,      // Hats
  true,   // X Axis
  true,   // Y Axis
  !altMode,  // Z Axis (disabled in T300RS mode)
  false,  // X Rotation
  false,  // Y Rotation
  altMode,   // Z Rotation (enabled in T300RS mode)
  false,  // Rudder
  false,  // Throttle
  false,  // Accelerator
  false,  // Break
  false   // Steering
);

int menuLength = 9;  // Total menu items
int operationMode = 0;
int oldOperationMode = 0;
int menuLevel = 0;
int oldMenuLevel = 0;
int lastMenuLevel = 1;
bool reopenLevel = false;

const int16_t joystickMin = -32767;
const int16_t joystickMax = 32767;

//PEDALS
float adjusted;

int16_t brakeSensor;
int16_t acceleratorSensor;

const int errorMargin = 7;
const int thresholds[7] =
{
  238, // button 3 (T)
  503, // button 5 (N)
  380, // button 6 (wiper)
  195, // buttons 3 & 5
  175, // buttons 3 & 6
  278, // buttons 5 & 6
  152  // buttons 3 & 5 & 6
}; 

int sensorValues[4];
bool buttons[15] = {false};
bool button01, button02, button03, button04, button05, button06, button07;
bool button08, button09, button10, button11, button12, button13, button14, button15;
bool buttonMenu;
bool oldButton01, oldButton02, oldButton03, oldButton04, oldButton05;
bool oldButton06, oldButton07, oldButton08, oldButton09, oldButton10;
bool oldButton11, oldButton12, oldButton13, oldButton14, oldButton15;
bool oldButtonMenu;
bool button3WasPressed, longPressTriggered;
long button3StartTime = 0;
bool button13WasPressed, button13AsFunction, button13OnHold;

volatile long encoderCount = 0; 
volatile long encoderCountParam = 0;    
long lastEncoderCountParam = 0;

void isrA() { encoderCount += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ? 1 : -1;
             encoderCountParam += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ? 1 : -1; }
void isrB() { encoderCount += (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) ? 1 : -1;
             encoderCountParam += (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) ? 1 : -1; }

float getAngleDeg() {
  long adjustedCount = encoderCount - encoderOffset;
  return (adjustedCount * 360.0f) / (float)PULSES_PER_REV;
}

// Force feedback effect parameter calculation
#define BUFLEN 4
int16_t posBuf[BUFLEN];
int16_t dtBuf[BUFLEN];
uint8_t pIdx = 0;

int16_t calculateEffectParams(unsigned long now, int16_t pos){
  static unsigned long lastEffectsUpdate = 0;
  static int16_t lastX = 0;
  static int16_t sum = 0;
  static int16_t sumdt = 0;

  params[0].springPosition = -pos;

  int16_t dt = now - lastEffectsUpdate;
  if(dt > 0){
    lastEffectsUpdate = now;
    sum   -= posBuf[pIdx];
    sumdt -= dtBuf[pIdx];
    int16_t d = pos - lastX;
    posBuf[pIdx] = d;
    dtBuf[pIdx]  = dt;
    sum   += d;
    sumdt += dt;
    pIdx = (pIdx + 1) % BUFLEN;
    lastX = pos;
    int32_t velNum = (int32_t)sum * 50;
    int16_t vel = velNum / (int32_t)sumdt;
    static int16_t lastVel = 0;
    int32_t dv = (int32_t)vel - lastVel;
    int16_t rawAcc = (dv * 1000) / dt;
    lastVel = vel;
    static int16_t accFilt = 0;
    accFilt = (accFilt * 8 + rawAcc * 2) / 10;
    int16_t acc = accFilt;
    params[0].frictionPositionChange = -vel;
    params[0].inertiaAcceleration    = acc;
    params[0].damperVelocity         = -vel;
  }

  Joystick.setEffectParams(params);
  return dt;
}

void motorDrive(float torque, float angle) {
  // HARD 900° LIMIT - Override everything if beyond range (BEFORE gain)
  if (angle < -450) {
    // Beyond left limit - force wheel RIGHT with maximum force
    torque = 200;  // Override with constant force
  } else if (angle > 450) {
    // Beyond right limit - force wheel LEFT with maximum force
    torque = -200;  // Override with constant force
  } else if (angle < -420 && torque < 0) {
    // Approaching left limit with leftward force - reduce it strongly
    torque *= 0.1f;
  } else if (angle > 420 && torque > 0) {
    // Approaching right limit with rightward force - reduce it strongly
    torque *= 0.1f;
  }

  // Apply global gain to all forces AFTER limit checks
  torque *= Gain;

  if (torque > PWM_MAX) torque = PWM_MAX;
  if (torque < -PWM_MAX) torque = -PWM_MAX;

  int pwm = (int)fabs(torque);

  // Small deadband for zero
  if (pwm < 3) {
    pwm = 0;
  }

  // NO MINIMUM PWM - let motor respond to small forces
  // The motor will handle small PWM values naturally

  if (pwm == 0) {
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    return;
  }

  if (torque >= 0) {
    analogWrite(PWM_A, pwm);
    analogWrite(PWM_B, 0);
  } else {
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, pwm);
  }
}


// SEND DATA TO JOYSTICK
void ProcessDataAndApply()
{
  float angle = getAngleDeg();
  // Don't constrain angle here - let it exceed ±450 to trigger hard limits
  // Only constrain for USB report to prevent overflow
  float reportAngle = constrain(angle, -450, 450);
  int16_t axis = map((long)reportAngle, -450, 450, -32767, 32767);
  axis = constrain(axis, -32767, 32767);

  Joystick.setXAxis(axis);

  // Calculate effect parameters for game FFB
  unsigned long now = millis();
  calculateEffectParams(now, axis);

  // GET FORCE FEEDBACK FROM GAME
  Joystick.getForce(forces);

  static float lastAngle = angle;
  float speed = angle - lastAngle;
  lastAngle = angle;

  float torque = 0.0f;

  // DISABLE MOTOR WHEN IN MENU - prevents crazy spinning
  if (operationMode == 1) {
    motorDrive(0, angle);
    return;
  }

  // GET GAME FORCE and invert if needed
  float gameForce = constrain(forces[0] * forceInvert, -200, 200);

  // Apply deadzone to prevent drift from small forces
  if (fabs(gameForce) < deadzone) {
    gameForce = 0.0f;
  }

  // DIRECT DRIVE MODE with optional damping
  torque = gameForce;

  // Add light damping to prevent free-spinning (adjustable)
  if (Bd > 0.0f) {
    float dampingForce = -speed * Bd;
    dampingForce = constrain(dampingForce, -50, 50);  // Limit damping strength
    torque += dampingForce;
  }

  // Hard limits applied in motorDrive function
  motorDrive(torque, angle);

  // BRAKE PEDAL (Y-Axis)
  int16_t brakeClamped = constrain(brakeSensor, brakeSensorMin, brakeSensorMax);
  brakeFinalValue = map((long)brakeClamped, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
  Joystick.setYAxis(brakeFinalValue);

  // ACCELERATOR PEDAL (Z-Axis or Rz-Axis in altMode)
  int16_t accelClamped = constrain(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax);
  acceleratorFinalValue = map((long)accelClamped, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);

  if (altMode) {
    Joystick.setRzAxis(acceleratorFinalValue);
  } else {
    Joystick.setZAxis(acceleratorFinalValue);
  }

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
  delay(1);

}


void EncoderParameterAdjust(float &value, float step, float minVal, float maxVal) {
  long delta = encoderCountParam - lastEncoderCountParam;
  if (abs(delta) >= 1) {
    value += delta * step;
    value = constrain(value, minVal, maxVal);
    lastEncoderCountParam = encoderCountParam;
  }
}

void ReadAnalogSensors()
{
  brakeSensor = ADS.readADC(0);
  acceleratorSensor = ADS.readADC(1);
}


void setup() {
  #if isFirstTimeUploading
    StoreData();
  #endif

  // USB Spoofing
  if(altMode){
     USBRename dummy = USBRename("d0z3r FF", "Thrustmaster", "T300RS", 0x044f, 0xb66e);
  }else{
    USBRename dummy = USBRename("d0z3r FF", "Logitech", "G29", 0x046d, 0xc24f);
  }

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  pinMode(14, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);

  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 399;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TIMSK3 |= (1 << OCIE3A);
  sei();


  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), isrB, CHANGE);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  DisplayMainScreen();

  Joystick.begin(true);
  Joystick.setXAxisRange(joystickMin, joystickMax);
  Joystick.setYAxisRange(joystickMin, joystickMax);
  if(altMode){
    Joystick.setRzAxisRange(joystickMin, joystickMax);
  }else{
    Joystick.setZAxisRange(joystickMin, joystickMax);
  }

  gains[0].totalGain = Gain;
  Joystick.setGains(gains);

  // Initialize effect parameters for game FFB
  params[0].springMaxPosition = joystickMax;
  params[0].damperMaxVelocity = 1500;
  params[0].inertiaMaxAcceleration = 8000;
  params[0].frictionMaxPositionChange = 1500;
  ReadData();
  delay(100);
  menuLevel = 0;
  operationMode = 0;

  ADS.begin();
  ADS.setMode(0);      // 0:continuous 1:single
  ADS.setGain(1);      // 0:6.144V  1:4.096V 2:2.048V 4:1.024V 8:0.512V 16:0.256V
  ADS.setDataRate(7);  // 0:slowest 4:default 7:fastest

    TCCR1B = TCCR1B & 0b11111000 | 0x01;

}

void loop() {
  ReadButtons();
  ReadAnalogSensors();
  if (operationMode == 0) {
    ProcessDataAndApply();
    if (buttonMenu) {
      operationMode = 1;
      oled.clear();
      menuLevel = lastMenuLevel;
    }
  }

  if (operationMode == 1) MenuOperations();

  Joystick.sendState();
  UpdateOldButtons();
  oldOperationMode = operationMode;

  wireInterface.endTransmission();
}

void MenuOperations() {
  if (button03 && !oldButton03 && menuLevel != 0) {
    menuLevel += 1;
    if (menuLevel > menuLength) menuLevel = 1;
  }
  
  if (button04 && !oldButton04 && menuLevel != 0) {
    DisplayConfirmationScreen();
    lastMenuLevel = menuLevel;
    menuLevel = 0;
  } else if (button04 && !oldButton04 && menuLevel == 0) {
      ReadData();
      DisplayMainScreen();
      operationMode = 0;
  } else if (button05 && !oldButton05 && menuLevel == 0) {
    
      StoreData();
      ReadData();
      gains[0].totalGain = Gain;
      Joystick.setGains(gains);

      DisplayMainScreen();
      operationMode = 0;
  }

  // Handle different menu levels
  if (menuLevel == 1) {
    if (menuLevel != oldMenuLevel){
      oled.clear();
    }
    oled.setRow(0); oled.setCol(0);
    oled.println(F("Calibration"));
    oled.println(F("Center wheel"));
    oled.print(F("Angle: "));
    oled.print(getAngleDeg(), 1);
    CleanLine();
    DisplayMenuSet();

    // Set center when button pressed
    if (button05 && !oldButton05) {
      encoderOffset = encoderCount;  // Set current position as center
    }
  }

  if (menuLevel == 2) {
    if (menuLevel != oldMenuLevel){
      oled.clear();
      lastEncoderCountParam = encoderCountParam;  // Reset encoder tracking
    }
    DisplayTitleForceFeedback();
    oled.print(F("Deadzone: "));
    CleanLineln();
    EncoderParameterAdjust(deadzone, 0.5, 0.0, 50.0);  // Range: 0-50
    oled.print(deadzone, 1);
    CleanLine();
    DisplayMenuChange();  // Changed from Set to Change
  }

  if (menuLevel == 3) {
    if (menuLevel != oldMenuLevel){
      oled.clear();
    }
    DisplayTitleForceFeedback();
    oled.print(F("Invert Force: "));
    if (forceInvert == 1) {
      oled.print(F("NO"));
    } else {
      oled.print(F("YES"));
    }
    CleanLine();
    DisplayMenuChange();

    // Toggle invert on button press
    if (button05 && !oldButton05) {
      forceInvert *= -1;  // Toggle between 1 and -1
    }
  }

  if (menuLevel == 4) {
    if (menuLevel != oldMenuLevel){
      oled.clear();
      lastEncoderCountParam = encoderCountParam;  // Reset encoder tracking
    }
    DisplayTitleForceFeedback();
    oled.print(F("FFB Gain: "));
    CleanLineln();
    EncoderParameterAdjust(Gain, 0.1, 0.1, 20.0);  // Range: 0.1-20
    oled.print(Gain, 1);
    CleanLine();
    DisplayMenuChange();  // Changed from Set to Change
  }


  if (menuLevel == 5) {
    if (menuLevel != oldMenuLevel) {
      oled.clear();
      lastEncoderCountParam = encoderCountParam;  // Reset encoder tracking
    }
    DisplayTitleForceFeedback();
    oled.print(F("Damping: "));
    CleanLineln();
    EncoderParameterAdjust(Bd, 1.0, 0.0, 30.0);
    oled.print(Bd, 1);
    CleanLine();
    DisplayMenuChange();  // Changed from Set to Change
  }

  if (menuLevel == 6) {
    if (menuLevel != oldMenuLevel) {
      oled.clear();
      lastEncoderCountParam = encoderCountParam;  // Reset encoder tracking
    }
    DisplayTitleForceFeedback();
    oled.print(F("Max Torque: "));
    CleanLineln();
    EncoderParameterAdjust(PWM_MAX, 5, 50, 250);  // Range: 50-250
    oled.print(PWM_MAX, 0);
    CleanLine();
    DisplayMenuChange();  // Changed from Set to Change
  }

  if (menuLevel == 7)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitlePedalCurve();
    oled.println(F("SteeringDegree"));
    oled.print("900");

    CleanLine();
    DisplayMenuChange();
  }


  if (menuLevel == 8) {
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
          DisplayTitleCalibration();
          oled.println(F("Release pedal"));
          oled.print(F("Minimum: "));
          break;
          
          case 1:
          brakeSensorMin = brakeSensor;
          brakePedalCalibrationStep++;
          DisplayTitleCalibration();
          oled.println(F("Press pedal"));
          oled.print(F("Maximum: "));
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

 if (menuLevel == 9)
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
          DisplayTitleCalibration();
          oled.println(F("Release pedal"));
          oled.print(F("Minimum: "));
          break;
        
        case 1:
          acceleratorSensorMin = acceleratorSensor;
          acceleratorPedalCalibrationStep++;
          DisplayTitleCalibration();
          oled.println(F("Press pedal"));
          oled.print(F("Maximum: "));
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

void CleanLine() { oled.print(F("    ")); }
void CleanLineln() { oled.println(F("    ")); }

void DisplayTitleCalibration() {
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("Calibration"));
}

void DisplayTitleForceFeedback() {
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("ForceFeedback"));
}

void DisplayTitleDegree(){
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("SteeringDegrees"));
}

void DisplayTitlePedalCurve() {
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.println(F("PedalCurve"));
}

void DisplayMainScreen() {
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.setLetterSpacing(0);
  oled.print(F("SP900F"));
  if(altMode){ oled.print(F(" ALT")); }
  oled.setLetterSpacing(1);
  oled.setRow(3); oled.setCol(85);
  oled.setInvertMode(true);
  oled.print(F(" Menu "));
  oled.setInvertMode(false);
  oled.print(F(">"));
}

void DisplayMenuBase() {
  oled.setRow(3); oled.setCol(0);
  oled.setInvertMode(true);
  oled.print(F(" Exit "));
  oled.setInvertMode(false);
}

void DisplayMenuSet() {
  DisplayMenuBase();
  oled.print(F("          "));
  oled.setInvertMode(true);
  oled.print(F(" Set"));
  oled.setInvertMode(false);
}

void DisplayMenuChange() {
  DisplayMenuBase();
  oled.print(F("       "));
  oled.setInvertMode(true);
  oled.print(F(" Change "));
  oled.setInvertMode(false);
}

void DisplayMenuStart() {
  DisplayMenuBase();
  oled.print(F("        "));
  oled.setInvertMode(true);
  oled.print(F(" Start "));
  oled.setInvertMode(false);
}

void DisplayConfirmationScreen() {
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.print(F("Save Change?"));
  oled.setRow(3); oled.setCol(0);
  oled.setInvertMode(true);
  oled.print(F(" No "));
  oled.setInvertMode(false);
  oled.print(F("            "));
  oled.setInvertMode(true);
  oled.print(F(" Yes "));
  oled.setInvertMode(false);
}


void ReadButtons() {
  int sensorValues[4];
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);

  for (int i = 0; i < 15; i++) buttons[i] = false;

  for (int i = 0; i < 4; i++) {
    if (abs(sensorValues[i] - thresholds[0]) <= errorMargin) { buttons[i * 3] = true; } // Button A
    if (abs(sensorValues[i] - thresholds[1]) <= errorMargin) { buttons[i * 3 + 1] = true; } // Button B
    if (abs(sensorValues[i] - thresholds[2]) <= errorMargin) { buttons[i * 3 + 2] = true; } // Button C
    if (abs(sensorValues[i] - thresholds[3]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 1] = true; } // A + B
    if (abs(sensorValues[i] - thresholds[4]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 2] = true; } // A + C
    if (abs(sensorValues[i] - thresholds[5]) <= errorMargin) { buttons[i * 3 + 1] = true; buttons[i * 3 + 2] = true; } // B + C
    if (abs(sensorValues[i] - thresholds[6]) <= errorMargin) { buttons[i * 3] = true; buttons[i * 3 + 1] = true; buttons[i * 3 + 2] = true; } // A + B + C
  }

  button01 = !digitalRead(16);
  button02 = !digitalRead(14);
  CheckButton3Press();
  
  button04 = buttons[6];
  button05 = buttons[10];
  button06 = buttons[11];
  button07 = buttons[7];
  button08 = buttons[8];
  if (!button13OnHold) button09 = buttons[3];
  button10 = buttons[4];
  if (!button13OnHold) button11 = buttons[5];
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

void UpdateOldButtons() {
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

void CheckButton13press() {
  if (button13) button13 = false;
  if (buttons[2]) {
    if (!button13OnHold) button13OnHold = true;
  } else if (!buttons[2] && button13OnHold) {
    button13OnHold = false;
    button14 = false;
    button15 = false;
    if (!button13AsFunction) button13 = true;
    button13AsFunction = false;
  }
}

void CheckButton3Press() {
  if (!buttons[9]) {
    if (oldButton03) button03 = false;
    if (oldButtonMenu) buttonMenu = false;
  }

  if (buttons[9]) {
    if (!button3WasPressed) {
      button3WasPressed = true;
      button3StartTime = millis();
      longPressTriggered = false;
    } else if (!longPressTriggered && (millis() - button3StartTime >= 1000)) {
      longPressTriggered = true;
      buttonMenu = true;
    }
  } else if (button3WasPressed) {
    if (!longPressTriggered) button03 = true;
    button3WasPressed = false;
    button3StartTime = 0;
    longPressTriggered = false;
  }
}

// TIMER3 INTERRUPTION
ISR(TIMER3_COMPA_vect)
{
  Joystick.getUSBPID();  // update FFB
}

void StoreData()
{
  EEPROM.put(0, Kc);
  EEPROM.put(4, Bd);
  EEPROM.put(8, Gain);

  EEPROM.put(12, brakeSensorMin);
  EEPROM.put(14, brakeSensorMax);

  EEPROM.put(16, acceleratorSensorMin);
  EEPROM.put(18, acceleratorSensorMax);

  EEPROM.put(20, PWM_MAX);
  EEPROM.put(24, encoderOffset);  // Save center calibration
  EEPROM.put(28, deadzone);  // Save deadzone
  EEPROM.put(32, forceInvert);  // Save force inversion

}
void ReadData()
{
  // Load damping from EEPROM - now adjustable in DD mode
  EEPROM.get(4, Bd);
  EEPROM.get(8, Gain);
  EEPROM.get(12, brakeSensorMin);
  EEPROM.get(14, brakeSensorMax);
  EEPROM.get(16, acceleratorSensorMin);
  EEPROM.get(18, acceleratorSensorMax);
  EEPROM.get(20, PWM_MAX);

  long tempOffset;
  EEPROM.get(24, tempOffset);
  // Validate encoderOffset - should be reasonable (within ±10 rotations = ±8400 pulses)
  if (tempOffset >= -8400 && tempOffset <= 8400) {
    encoderOffset = tempOffset;
  } else {
    encoderOffset = 0;  // Reset to 0 if garbage
  }

  EEPROM.get(28, deadzone);
  EEPROM.get(32, forceInvert);

  // Validate parameters
  if (Gain < 0.1f || Gain > 20.0f) Gain = 1.2f;
  if (PWM_MAX < 50 || PWM_MAX > 255) PWM_MAX = 250;
  if (Bd < 0.0f || Bd > 30.0f) Bd = 0.0f;
  if (deadzone < 0.0f || deadzone > 50.0f) deadzone = 15.0f;
  if (forceInvert != 1 && forceInvert != -1) forceInvert = 1;
}
