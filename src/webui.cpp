#include "blinds_core.h"
#include "webui.h"
#include <Arduino.h> // For Serial
#include <MycilaWebSerial.h> // For webSerial
#include <Preferences.h>


// Declare webSerial as extern if it's defined in main.cpp
extern WebSerial webSerial;
// Declare Preferences object as extern if it's defined elsewhere
extern Preferences preferences;
// Declare stepperAcceleration as extern since it's defined elsewhere
extern uint32_t stepperSpeed;
extern uint32_t stepperAcceleration;

// Declare functions from main.cpp as extern
extern void runForward();
extern void stopMotors();


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
void speedApplyCallback(Control* sender, int type)  
{
    switch (type)
    {
    
    case B_UP:
        // Handle button release
        uint32_t newSpeed = ESPUI.getControl(speedControl)->value.toInt();
        setSteppersSpeed(newSpeed);
        break;
    }
}

void speedSaveCallback(Control* sender, int type)
{
    switch (type)
    {
    
    case B_UP:
        // Handle button release
        uint32_t newSpeed = ESPUI.getControl(speedControl)->value.toInt();
        webSerial.printf("Speed value is  %u \n", newSpeed);
        setSteppersSpeed(newSpeed);
        preferences.begin("blinds", false);
        preferences.putLong("speed", stepperSpeed);
        preferences.end();
        break;
    }
}

void accelerationApplyCallback(Control* sender, int type)
{
    switch (type)
    {
    
    case B_UP:
        // Handle button release
        uint32_t newAcceleration = ESPUI.getControl(accelControl)->value.toInt();
        webSerial.printf("Acceleration value is  %u \n", newAcceleration);
        setSteppersAcceleration(newAcceleration);
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
        setSteppersAcceleration(newAcceleration);
        preferences.begin("blinds", false);
        preferences.putLong("acceleration", stepperAcceleration);
        preferences.end();
        break;
    }
}

void forwardCallback(Control* sender, int type) {
    runForward();
}

void stopCallback(Control* sender, int type) {
    stopMotors();
}

// Add more callback function implementations here as needed


// This is the main function which builds our GUI
void setupUI() {

    String clearLabelStyle = "background-color: unset; width: 100%;";
    String switcherLabelStyle = "width: 60px; margin-left: .3rem; margin-right: .3rem; background-color: unset;";

    auto grouptab = ESPUI.addControl(Tab, "", "Status");
    auto vertgroupslider = ESPUI.addControl(Slider, "Vertical Slider Group", "0", Dark, grouptab, generalCallback);
	ESPUI.setVertical(vertgroupslider);
	ESPUI.setVertical(ESPUI.addControl(Slider, "", "100", None, vertgroupslider, generalCallback));
	
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "", None, vertgroupslider), clearLabelStyle);
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, vertgroupslider), switcherLabelStyle);
	ESPUI.setElementStyle(ESPUI.addControl(Label, "", "T", None, vertgroupslider), switcherLabelStyle);

    auto settingstab = ESPUI.addControl(Tab, "", "Settings");
    speedControl = ESPUI.addControl(Number, "Speed", String(stepperSpeed), ControlColor::Dark, settingstab , numberCallback);
    ESPUI.addControl(Min, "", "0", None, speedControl);
    ESPUI.addControl(Max, "", "30000", None, speedControl);
    ESPUI.addControl(Button, "Apply", "Apply", ControlColor::Dark, speedControl, speedApplyCallback);
    ESPUI.addControl(Button, "Save", "Save", ControlColor::Dark, speedControl, speedSaveCallback);

    accelControl = ESPUI.addControl(Number, "Acceleration", String(stepperAcceleration), ControlColor::Dark, settingstab , numberCallback);
    ESPUI.addControl(Min, "", "0", None, accelControl);
    ESPUI.addControl(Max, "", "800000", None, accelControl);
    ESPUI.addControl(Button, "Apply", "Apply", ControlColor::Dark, accelControl, accelerationApplyCallback);
    ESPUI.addControl(Button, "Save", "Save", ControlColor::Dark, accelControl, accelerationSaveCallback);

    ESPUI.addControl(Button, "forward", "forward", ControlColor::Dark, settingstab, forwardCallback);
    ESPUI.addControl(Button, "stop", "stop", ControlColor::Alizarin, settingstab, stopCallback);
}