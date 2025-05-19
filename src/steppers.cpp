#include "stepper_interface.h"
#include <Preferences.h>
#include <MycilaWebSerial.h>
#include "configLoader.h"
#include <FastAccelStepper.h>

// External references
extern Preferences preferences;
extern WebSerial webSerial;

// Add definitions for these arrays
bool currentlyHomingStepper[2] = {false, false};
bool sinceStartupHomedStepper[2] = {false, false};

struct stepper_config_s stepper_config;

FastAccelStepper* stepper[2];  // Array of pointers to FastAccelStepper objects
FastAccelStepperEngine engine;

void loadStoredStepperValues() {
  //check if we can load stored stepper values
  
  stepper_config.speed = preferences.getLong("speed", defaultSpeed);
  stepper_config.acceleration = preferences.getLong("acceleration", defaultAcceleration);
  stepper_config.homingSpeed = preferences.getLong("homingSpeed", defaultHomingSpeed);
  stepper_config.calibrationSpeed = preferences.getLong("calibrationSpeed", defaultCalibrationSpeed);
  stepper_config.calibrationAcceleration = preferences.getLong("calibrationAcceleration", defaultCalibrationAcceleration);
  stepper_config.safetyMargin = preferences.getLong("safetyMargin", defaultSafetyMargin);

  long savedStepper0 = preferences.getLong("maxStepper0", 0);
  if (savedStepper0 != 0) {
    maxPositionStepper[0] = savedStepper0;
    calibratedStepper[0] = true;
    webSerial.printf("stepper0 calibration found: %li", maxPositionStepper[0]);
  }
  
  long savedStepper1 = preferences.getLong("maxStepper1", 0);
  if (savedStepper1 != 0) {
    maxPositionStepper[1] = savedStepper1;
    calibratedStepper[1] = true;
    webSerial.printf("stepper1 calibration found: %li", maxPositionStepper[1]);
  }
  
  //if (calibratedStepper[0] == true && calibratedStepper[1] == true){
  if (calibratedStepper[0] == true) {
    calibrated = true;
  }
  
  // ##### end load calibration ######
}


void initializeSteppers() {
  // ---------- initialize stepper engine -------------
  engine.init();
  
  //stepper0
  FastAccelStepper *s = engine.stepperConnectToPin(stepPinStepper0, DRIVER_MCPWM_PCNT);
  bool activeLow = !reverseStepper0;
  s->setDirectionPin(dirPinStepper0, activeLow); // Pass pin AND activeLow setting
  s->setSpeedInHz(stepper_config.speed);
  s->setAcceleration(stepper_config.acceleration);
  stepper[0] = s;

  //stepper1
  s = engine.stepperConnectToPin(stepPinStepper1, DRIVER_MCPWM_PCNT);
  activeLow = !reverseStepper1;
  s->setDirectionPin(dirPinStepper1, activeLow); // Pass pin AND activeLow setting
  s->setAcceleration(stepper_config.acceleration);
  stepper[1] = s;
}

void startCalibration(){
  calibrating = true;
  calibratedStepper[0] = false;
  calibratedStepper[1] = false;

  stepper[0]->setSpeedInHz(stepper_config.calibrationSpeed);
  stepper[0]->setAcceleration(stepper_config.calibrationAcceleration);
  stepper[0]->runForward();
  
  //stepper[1]->setSpeedInHz(stepper_config.homingSpeed);
  //stepper[1]->runForward();
}


void finishCalibrateStepper(uint8_t stepperId){
  // we are now back at start position.
  // Current position will be 0 and negative steps traveled will be our max position.
  stepsTraveledStepper[stepperId] = stepper[stepperId]->getCurrentPosition();
  stepper[stepperId]->setCurrentPosition(0);
  maxPositionStepper[stepperId] = stepsTraveledStepper[stepperId];
  calibratedStepper[stepperId] = true;

  //also we're homed now
  homedStepper[stepperId] = true;
  currentlyHomingStepper[stepperId] = false;
  sinceStartupHomedStepper[stepperId] = true;
  
  if (stepperId == 0){
    preferences.putLong("maxStepper0",maxPositionStepper[0]);
    webSerial.printf("saved maxStepper0: %li",maxPositionStepper[0]);
  }
  if (stepperId == 1){
    preferences.putLong("maxStepper1",maxPositionStepper[1]);
    webSerial.printf("saved maxStepper1: %li",maxPositionStepper[1]);
  }

  // we're ready to rumble: set stepper to fullspeed
  stepper[stepperId]->setSpeedInHz(stepper_config.speed);
}

void startHomingSteppers(bool force) {
  if (!force){
    return;
  }
  currentlyHomingStepper[0] = true;
  currentlyHomingStepper[1] = true;
  stepper[0]->setSpeedInHz(stepper_config.homingSpeed);
  stepper[0]->runBackward();

  //stepper[1]->setSpeedInHz(stepper_config.homingSpeed);
  //stepper[1]->runBackward();
  webSerial.print(">>> starting homing ...");
}
  
void finishHomingStepper(uint8_t stepperId){
  stepper[stepperId]->stopMove();
  stepper[stepperId]->setCurrentPosition(0);
  homedStepper[stepperId] = true;
  currentlyHomingStepper[stepperId] = false;
  sinceStartupHomedStepper[stepperId] = true;
  webSerial.printf(">>> Homing stepper %li complete!", stepperId);
  stepper[stepperId]->setSpeedInHz(stepper_config.speed);
}
  


void saveSteppersAcceleration(uint32_t acceleration) {
  stepper_config.acceleration = acceleration;
  preferences.putLong("acceleration", acceleration);
  stepper[0]->setAcceleration(acceleration);
  stepper[1]->setAcceleration(acceleration);
}

void saveSteppersSpeed(uint32_t speed) {
  stepper_config.speed = speed;
  preferences.putLong("speed", speed);
  stepper[0]->setSpeedInHz(speed);
  stepper[1]->setSpeedInHz(speed);
}
void saveSafetyMargin(uint32_t safetyMargin) {
  stepper_config.safetyMargin = safetyMargin;
  preferences.putLong("safetyMargin", safetyMargin);
}


void runForward() {
  stepper[0]->runForward();
}

void stopMotors() {
  stepper[0]->stopMove();
  stepper[1]->stopMove();
}

void onLimitSwitchPressed(int buttonId){
  webSerial.printf("Button %d pressed", buttonId);
  Serial.println(buttonId);
  switch (buttonId) {
    case 1:
    {
      //limitStartStepper0
      stepper[0]->stopMove();
      webSerial.println("limitStartStepper0 pressed");
      currentlyHomingStepper[0] = true;
      stepper[0]->setSpeedInHz(stepper_config.homingSpeed/10);
      stepper[0]->runForward();
      break;
    }
    case 2:
    {
      //limitEndStepper0
      stepper[0]->stopMove();
      webSerial.println("limitEndStepper0 pressed");
      stepper[0]->setSpeedInHz(stepper_config.homingSpeed/10);
      stepper[0]->runBackward();
      break;
    }
    case 3:
    {
      //limitStartStepper1
      stepper[1]->stopMove();
      currentlyHomingStepper[1] = true;
      stepper[1]->setSpeedInHz(stepper_config.homingSpeed/10);
      stepper[1]->runForward();
      break;
    }
    case 4:
    {
      //limitEndStepper1
      stepper[1]->stopMove();
      stepper[1]->setSpeedInHz(stepper_config.homingSpeed/10);
      stepper[1]->runBackward();
      break;
    }
  }
}

void onLimitSwitchReleased(int buttonId){
  switch (buttonId) {
    case 1:
    {
      //limitStartStepper0
      stepper[0]->stopMove();
      if (currentlyHomingStepper[0]){
        finishHomingStepper(0);
      }
      
      if (calibrating && !calibratedStepper[0]){
        finishCalibrateStepper(0);
      }
      break;
    }
    case 2:
    {
      //limitEndStepper0
      stepper[0]->stopMove();
      if (calibrating){
        stepper[0]->setCurrentPosition(0);
        stepper[0]->setSpeedInHz(stepper_config.calibrationSpeed);
        stepper[0]->runBackward();
      }
      break;
    }
    case 3:
    {
      //limitStartStepper1
      stepper[1]->stopMove();
      
      if (currentlyHomingStepper[1]){
        finishHomingStepper(1);
      }

      if (calibrating && !calibratedStepper[1]){
        finishCalibrateStepper(1);
      }
      break;
    }
    case 4:
    {
      //limitEndStepper1
      stepper[1]->stopMove();
      if (calibrating && !calibratedStepper[1]){
        stepper[1]->setCurrentPosition(0);
        stepper[1]->setSpeedInHz(stepper_config.homingSpeed);
        stepper[1]->runBackward();
      }
      break;
    }
  }
}

void moveScreenSafelyFromNormalizedPosition(uint8_t stepperId, float normalizedPosition) {
  if (calibratedStepper[stepperId] == false){
    webSerial.println(">>> stepper not yet calibrated, OSC positions ignored");
    return;
  }
  if (sinceStartupHomedStepper[stepperId] == false){
    webSerial.println(">>> stepper not yet homed, OSC positions ignored");
    return;
  } 
  if (normalizedPosition < 0.0 || normalizedPosition > 1.0){
    webSerial.println(">>> invalid OSC position, must be between 0 and 1");
    return;
  }
  webSerial.printf(">>> input value: %f", normalizedPosition);
  // Safety margin
  long safetyMargin = stepper_config.safetyMargin;
  webSerial.printf(">>> safety margin: %li", safetyMargin);
  long safetyMinimum = 0 + safetyMargin;
  long safetyMaximum = maxPositionStepper[stepperId] - safetyMargin;
  long remappedValue = (long)(normalizedPosition * (float)(safetyMaximum - safetyMinimum) + safetyMinimum);
  webSerial.printf(">>> remapped value: %li", remappedValue);
  stepper[stepperId]->moveTo(remappedValue);
}

