#include "webserial.h"
#include "configLoader.h"
#include "stepper_interface.h"
#include <ESPUI.h>

// External references
extern WebSerial webSerial;
extern const char* DNSName;
extern bool calibrating;
extern bool calibratedStepper[2];
extern bool calibrated;
extern bool homingDoneSinceStartup;

void setupWebSerialCommands() {
  webSerial.onMessage([](const std::string& msg) 
  { 
    Serial.println(msg.c_str()); 
    webSerial.printf("> received command: %s", msg.c_str());
    String ms = msg.c_str();

    // if(msg == "status"){ 
    //     sendStatus();
    // }
    // else if(msg == "home"){ 
    //     startHomingSteppers(true);
    // }
    if(msg == "high"){ 
        testHigh();
    }
    else if(msg == "low"){ 
        testLow();
    }
    else{
      webSerial.printf("> error: unknown command");
    }
  });
}

void testHigh(){
  webSerial.print(">>> test high...");
  // Using dirPinStepper1 (14) and stepPinStepper1 (15)
  pinMode(dirPinStepper1, OUTPUT);
  pinMode(stepPinStepper1, OUTPUT);
  digitalWrite(dirPinStepper1, HIGH);
  digitalWrite(stepPinStepper1, HIGH);
}

void testLow(){
  webSerial.print(">>> test low...");
  // Using dirPinStepper1 (14) and stepPinStepper1 (15)
  pinMode(dirPinStepper1, OUTPUT);
  pinMode(stepPinStepper1, OUTPUT);
  digitalWrite(dirPinStepper1, LOW);
  digitalWrite(stepPinStepper1, LOW);
}

// void sendStatus(){
//   webSerial.println("");
//   webSerial.printf("####### status %s ####", DNSName);
//   webSerial.printf("calibrating:  %s", calibrating ? "true" : "false");
//   webSerial.printf("calibratedStepper[0]: %s", calibratedStepper[0] ? "true" : "false"); 
//   webSerial.printf("calibratedStepper[1]: %s", calibratedStepper[1] ? "true" : "false");
//   webSerial.printf("calibrated:  %s", calibrated ? "true" : "false");
//   webSerial.printf("homingDoneSinceStartup:  %s", homingDoneSinceStartup ? "true" : "false");
//   webSerial.print("##########################");
// }