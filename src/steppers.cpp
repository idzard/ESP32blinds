#include "stepper_interface.h"
#include <Preferences.h>
#include <MycilaWebSerial.h>
#include "configLoader.h"

// External references
extern Preferences preferences;
extern WebSerial webSerial;
extern const char* DNSName;


void loadStoredStepperValues() {
  //check if we can load stored stepper values
  
  stepper_config.speed = preferences.getLong("speed", defaultSpeed);
  stepper_config.acceleration = preferences.getLong("acceleration", defaultAcceleration);
  stepper_config.homingSpeed = preferences.getLong("homingSpeed", defaultHomingSpeed);
  stepper_config.calibrationSpeed = preferences.getLong("calibrationSpeed", defaultCalibrationSpeed);
  stepper_config.calibrationAcceleration = preferences.getLong("calibrationAcceleration", defaultCalibrationAcceleration);
  
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
  if (steppersHomed && !force){
    return;
  }
  homing = true;
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
  webSerial.printf(">>> Done homing stepper: %i", stepperId);
  //if (homedStepper[0] ==true && homedStepper[1] == true){
  if (homedStepper[0] ==true){
    steppersHomed = true;
    homing = false;
    homingDoneSinceStartup = true;
    webSerial.print(">>> Homing complete!");
    stepper[0]->setSpeedInHz(stepper_config.speed);
    stepper[1]->setSpeedInHz(stepper_config.speed);
  }
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

void runForward() {
  stepper[0]->runForward();
}

void stopMotors() {
  stepper[0]->stopMove();
  stepper[1]->stopMove();
}