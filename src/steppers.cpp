#include "stepper_interface.h"
#include <Preferences.h>
#include <MycilaWebSerial.h>
#include "configLoader.h"
#include <FastAccelStepper.h>

// External references
extern Preferences preferences;
extern WebSerial webSerial;


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
  stepsTraveledStepper[stepperId] = -1 * stepper[stepperId]->getCurrentPosition();
  stepper[stepperId]->setCurrentPosition(0);
  maxPositionStepper[stepperId] = stepsTraveledStepper[stepperId];
  calibratedStepper[stepperId] = true;
  
  //if (calibratedStepper[0] == true && calibratedStepper[1] == true){
  if (calibratedStepper[0] == true){
    //all calibration done
    
    // Store the position and close the Preferences
    preferences.putLong("maxStepper0",maxPositionStepper[0]);
    
    webSerial.printf("saved maxStepper0: %li",maxPositionStepper[0]);
    //preferences.putUInt("maxPositionStepper1",maxPositionStepper[1]);

    preferences.end();
    
    calibrating = false;
    calibrated = true;
    // we're ready to rumble: set steppers to fullspeed
    stepper[0]->setSpeedInHz(stepper_config.speed);
    stepper[1]->setSpeedInHz(stepper_config.speed);
    webSerial.printf(">>> Calibration of %s done!", DNSName);
  }
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
      Serial.println("limitStartStepper0 released");
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

void moveScreenSafelyFromNormalizedPosition(uint8_t stepperId, float value){
  if (calibratedStepper[stepperId] == false){
    webSerial.println(">>> stepper not calibrated, OSC positions ignored");
    return;
  } 
  else {
    if (value < 0.0 || value > 1.0){
      webSerial.println(">>> invalid OSC position, must be between 0 and 1");
      return;
    } 
  }
  webSerial.printf(">>> input value: %f", value);
  // Safety margin
  long safetyMargin = stepper_config.safetyMargin;
  webSerial.printf(">>> safety margin: %li", safetyMargin);
  long safetyMinimum = 0 + safetyMargin;
  long safetyMaximum = maxPositionStepper[stepperId] - safetyMargin;
  long remappedValue = (long)(value * (float)(safetyMaximum - safetyMinimum) + safetyMinimum);
  webSerial.printf(">>> remapped value: %li", remappedValue);
  stepper[stepperId]->moveTo(remappedValue);
}

