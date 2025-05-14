#include "blinds_core.h"
#include "stepper_interface.h"
#include "webui.h"
#include <Arduino.h> // For Serial
#include <MycilaWebSerial.h> // For webSerial
#include <Preferences.h>



extern WebSerial webSerial;

extern Preferences preferences;


extern bool calibratedStepper[2];
extern struct stepper_config_s stepper_config;


// Declare functions from main.cpp as extern
extern void runForward();
extern void stopMotors();
extern void startCalibration();
extern void startHomingSteppers(bool force);
extern void saveSteppersSpeed(uint32_t speed);
extern void saveSteppersAcceleration(uint32_t acceleration);

//UI handles
uint16_t speedControl, accelControl;

void textCallback(Control *sender, int type);
void generalCallback(Control *sender, int type);

void numberCallback(Control *sender, int type){
    uint32_t value = sender->value.toInt();
    webSerial.printf("new value is  %u \n", value);
}


void generalCallback(Control *sender, int type) {
	Serial.print("CB: id(");
	Serial.print(sender->id);
	Serial.print(") Type(");
	Serial.print(type);
	Serial.print(") '");
	Serial.print(sender->label);
	Serial.print("' = ");
	Serial.println(sender->value);
}


void speedSaveCallback(Control* sender, int type)
{
    switch (type)
    {
    
    case B_UP:
        // Handle button release
        uint32_t newSpeed = ESPUI.getControl(speedControl)->value.toInt();
        webSerial.printf("Speed value is  %u \n", newSpeed);
        saveSteppersSpeed(newSpeed);
        break;
    }
}



void accelerationSaveCallback(Control* sender, int type)
{
    switch (type)
    {
    
    case B_UP:
        // Handle button release
        uint32_t newAcceleration = ESPUI.getControl(accelControl)->value.toInt();
        webSerial.printf("Acceleration value is  %u \n", newAcceleration);
        saveSteppersAcceleration(newAcceleration);
        
        break;
    }
}

void forwardCallback(Control* sender, int type) {
    runForward();
}

void stopCallback(Control* sender, int type) {
    stopMotors();
}

void calibrateCallback(Control* sender, int type) {
    startCalibration();
}

void homingCallback(Control* sender, int type) {
    startHomingSteppers(true);
}


// Add more callback function implementations here as needed


// This is the main function which builds our GUI
void setupUI() {

    String clearLabelStyle = "background-color: unset; width: 100%;";
    String switcherLabelStyle = "width: 60px; margin-left: .3rem; margin-right: .3rem; background-color: unset;";

    // --------------------- Control tab ---------------------
    auto controlstab = ESPUI.addControl(Tab, "", "Control");
    auto vertgroupslider = ESPUI.addControl(Slider, "Positions", "0", Dark, controlstab, generalCallback);
	ESPUI.setVertical(vertgroupslider);
	ESPUI.setVertical(ESPUI.addControl(Slider, "", "100", None, vertgroupslider, generalCallback));
	
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "", None, vertgroupslider), clearLabelStyle);
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, vertgroupslider), switcherLabelStyle);
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "T", None, vertgroupslider), switcherLabelStyle);

    
    auto calibratedStatus = ESPUI.addControl(Label, "Calibration", "Not calibrated yet!", Dark, controlstab);
    ESPUI.updateLabel(calibratedStatus, calibratedStepper[0] ? "Frame is calibrated" : "Not calibrated yet!");
    ESPUI.addControl(Button, "Start calibration", "Start calibration", ControlColor::Dark, calibratedStatus, calibrateCallback);

    ESPUI.addControl(Button, "Start homing", "Start homing", ControlColor::Dark, controlstab, homingCallback);

    // --------------------- Settings tab ---------------------
    auto settingstab = ESPUI.addControl(Tab, "", "Settings");
    speedControl = ESPUI.addControl(Number, "Speed", String(stepper_config.speed), ControlColor::Dark, settingstab , numberCallback);
    ESPUI.addControl(Min, "", "0", None, speedControl);
    ESPUI.addControl(Max, "", "30000", None, speedControl);
    ESPUI.addControl(Button, "Save", "Save", ControlColor::Dark, speedControl, speedSaveCallback);

    accelControl = ESPUI.addControl(Number, "Acceleration", String(stepper_config.acceleration), ControlColor::Dark, settingstab , numberCallback);
    ESPUI.addControl(Min, "", "0", None, accelControl);
    ESPUI.addControl(Max, "", "800000", None, accelControl);
    ESPUI.addControl(Button, "Save", "Save", ControlColor::Dark, accelControl, accelerationSaveCallback);

    ESPUI.addControl(Button, "forward", "forward", ControlColor::Dark, settingstab, forwardCallback);
    ESPUI.addControl(Button, "stop", "stop", ControlColor::Alizarin, settingstab, stopCallback);
}