#ifndef STEPPER_INTERFACE_H
#define STEPPER_INTERFACE_H

#include <Arduino.h>
#include <FastAccelStepper.h> // For FastAccelStepper type

// Pin Definitions
#define stepPinStepper1 15
#define dirPinStepper1 14
#define stepPinStepper2 32
#define dirPinStepper2 33
#define DRIVER_MCPWM_PCNT 0

// Struct Definitions
struct stepper_config_s {
  uint8_t step;
  uint8_t direction;
  uint32_t speed;
  uint32_t homingSpeed;
  uint32_t acceleration;
};

// Extern Variable Declarations for Stepper Control
extern long m1Max; // Max position for motor 1 (derived from calibration)
extern long m2Max; // Max position for motor 2 (derived from calibration)

extern uint32_t stepperSpeed;
extern uint32_t stepperAcceleration;

extern bool homing;
extern bool homingDoneSinceStartup;
extern bool homedStepper[2];
extern bool steppersHomed;
extern bool calibrating;
extern bool calibrated;
extern bool calibratedStepper[2];

extern long stepsTraveledStepper[2];
extern uint32_t minPositionStepper[2];
extern long maxPositionStepper[2];

extern const struct stepper_config_s stepper_config[2];
extern FastAccelStepper *stepper[2];
extern FastAccelStepperEngine engine;

// External variables from configLoader.h used in stepper configuration
extern bool reverseStepper0;
extern bool reverseStepper1;

// Function Prototypes for Stepper Control
void startHomingSteppers(bool force); // No default argument
void startCalibration();
void finishHomingStepper(uint8_t stepperId);
void finishCalibrateStepper(uint8_t stepperId);

#endif // STEPPER_INTERFACE_H