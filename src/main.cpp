#include <Arduino.h>
#include <ArtnetETH.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ESPUI.h>
#include <ezButton.h>
#include <FastAccelStepper.h>
#include <MycilaWebSerial.h>
#include <Preferences.h>
#include "configLoader.h"
#include "stepper_interface.h"
#include "blinds_core.h"
#include "webui.h"

WebSerial webSerial;

ezButton button1(4);
ezButton button2(35);


long m1Max = 3200;
long m2Max = 3200;

// these fallback values are used if no saved preferences are found
uint32_t defaultStepperSpeed = 30000;
uint32_t defaultStepperAcceleration = 80000;
uint32_t defaultStepperHomingSpeed = 5000;
uint32_t defaultCalibrationSpeed = 15000;
uint32_t defaultCalibrationAcceleration = 800000;

uint32_t stepperSpeed = defaultStepperSpeed;
uint32_t stepperAcceleration = defaultStepperAcceleration;

bool homing = false;
bool homingDoneSinceStartup = false;
bool homedStepper[2] = {false, false};
bool steppersHomed = false;
bool calibrating = false;
bool calibrated = false;
bool calibratedStepper[2] = {false, false};

long stepsTraveledStepper[2];
uint32_t minPositionStepper[2] = {0,0};
long maxPositionStepper[2];

// Using the stepper_config_s struct defined in stepper_interface.h
const struct stepper_config_s stepper_config[2] = {
    {
      step : stepPinStepper1,
      direction : dirPinStepper1,
    },
    {
      step : stepPinStepper2,
      direction : dirPinStepper2,
    }};


int statusLabelId;
int graphId;
int millisLabelId;
int testSwitchId;
int oldTime = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[2];

Preferences preferences;

ArtnetReceiver artnet;
uint16_t universe1 = 1; // 0 - 32767
uint8_t net = 0;        // 0 - 127
uint8_t subnet = 0;     // 0 - 15
uint8_t universe2 = 2;  // 0 - 15

unsigned long lastHeartbeat = 0;




void setup() {
  Serial.begin(115200);
  
  if (!loadConfiguration()){
    Serial.println("config loading fucked");
  } else{
    Serial.println("config loaded");
  };
  
  button1.setDebounceTime(50); // set debounce time to 50 milliseconds
  button2.setDebounceTime(50); // set debounce time to 50 milliseconds


  

  // Open Preferences with 'blinds' namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("blinds", false);
  
  ETH.begin();
  ETH.config(ip, gateway, subnet_mask);
  MDNS.begin(DNSName); 


  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin(); 




  
  //check if we can load stored stepper values
  
  uint32_t savedSpeed = preferences.getLong("speed", -1);
  if (savedSpeed != -1){
    stepperSpeed = savedSpeed;

  }
  uint32_t savedAcceleration = preferences.getLong("acceleration", -1);
  if (savedAcceleration != -1){
    stepperAcceleration = savedAcceleration;

  }



  uint32_t savedStepper0 = preferences.getLong("maxStepper0", 0);
  if (savedStepper0 != 0){
    maxPositionStepper[0] = savedStepper0;
    calibratedStepper[0] = true;
    webSerial.printf("stepper0 calibration found: %li", maxPositionStepper[0]);
  }
  uint32_t savedStepper1 = preferences.getLong("maxStepper1", 0);
  if (savedStepper1 != 0){
    maxPositionStepper[1] = savedStepper1;
    calibratedStepper[1] = true;
    webSerial.printf("stepper1 calibration found: %li", maxPositionStepper[1]);
  }
  //if (calibratedStepper[0] == true && calibratedStepper[1] == true){
  if (calibratedStepper[0] == true){
      calibrated = true;
  }

  // ##### end load calibration ######
  
  
  engine.init();
  for (uint8_t i = 0; i < 2; i++) {
    FastAccelStepper *s = NULL;
    const struct stepper_config_s *config = &stepper_config[i];
    if (config->step != PIN_UNDEFINED) {
      s = engine.stepperConnectToPin(config->step, DRIVER_MCPWM_PCNT);
      if (s) {
        // Determine the correct reverse setting using if-else
        bool reverse;
        if (i == 0) {
          reverse = reverseStepper0; // Use setting for stepper 0
        } else {
          reverse = reverseStepper1; // Use setting for stepper 1
        }

        // Determine the activeLow value for the library function
        bool activeLow = !reverse; // Invert the config flag (true reversed -> false activeLow)

        s->setDirectionPin(config->direction, activeLow); // Pass pin AND activeLow setting
        s->setSpeedInHz(stepperSpeed);
        s->setAcceleration(stepperAcceleration);
      }
    }
    stepper[i] = s;
  }
  


  webSerial.onMessage([](const std::string& msg) 
  { 
    Serial.println(msg.c_str()); 
    webSerial.printf("> received command: %s", msg.c_str());
    String ms = msg.c_str();
    if (msg == "calibrate"){
      startCalibration();
      webSerial.printf(">>> Starting calibration of %s ...", DNSName);
    } else if(msg == "status"){ 
        sendStatus();
    }
    else if(msg == "home"){ 
        startHomingSteppers(true);
    }
    else if(msg == "high"){ 
        testHigh();
    }
    else if(msg == "low"){ 
        testLow();
    }
    else{
      webSerial.printf("> error: unknown command");
    }
  });
 


  webSerial.printf(">>>>>> %s is online. Searching for saved calibration...", DNSName);


  setupUI();
  ESPUI.setVerbosity(Verbosity::VerboseJSON);

  // Create a static buffer for the webpage title
  static char titleBuffer[64];
  strcpy(titleBuffer, DNSName);
  strcat(titleBuffer, " Control");
  ESPUI.begin(titleBuffer);
  webSerial.begin(ESPUI.WebServer()); // Initialize WebSerial with ESPUI's server
  
  // #####  end OTA & WebSocket server ######


  artnet.begin();
  artnet.subscribeArtDmxUniverse(net, subnet, universe1, onArtnetReceive);

  
  startHomingSteppers(true);
}


// ####### START CALIBRATION #################

void startCalibration(){
  calibrating = true;
  calibratedStepper[0] = false;
  calibratedStepper[1] = false;

  stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
  stepper[0]->runBackward();
  
  //stepper[1]->setSpeedInHz(stepper_config[1].homingSpeed);
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
    stepper[0]->setSpeedInHz(stepper_config[0].speed);
    stepper[1]->setSpeedInHz(stepper_config[1].speed);
    webSerial.printf(">>> Calibration of %s done!", DNSName);

  }
}



void onLimitSwitchPressed(int buttonId){
  webSerial.printf("Button %d pressed", buttonId);
  Serial.println(buttonId);
  switch (buttonId) {
    case 1:
    {
      //limitStartStepper0
      stepper[0]->stopMove();
      Serial.println("limitStartStepper0 pressed");
      if (homing){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runBackward();
        Serial.println("slow homing forward");
      }
      
      
      if (calibrating && !calibratedStepper[0]){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runBackward();
      }
      break;  
    }
    case 2:
    {
      //limitEndStepper0
      stepper[0]->stopMove();
      Serial.println("limitEndStepper0 pressed");
      if (calibrating && !calibratedStepper[0]){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runForward();
      }
      break;  
    }
    case 3:
    {
      //limitStartStepper1
      stepper[1]->stopMove();
      
      if (homing){
        stepper[1]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[1]->runForward();
        Serial.println("limitStartStepper1 pressed. slow homing forward");
      }
      
      if (calibrating && !calibratedStepper[1]){
        stepper[1]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[1]->runForward();
      }
      break;  
    }
    case 4:
    {
      //limitEndStepper1
      stepper[1]->stopMove();
      Serial.println("limitEndStepper1 pressed");
      if (calibrating && !calibratedStepper[1]){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runBackward();
         
      }
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
      if (homing){
        Serial.println("homing done");
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
      Serial.println("limitEndStepper0 released");
      if (calibrating){
        stepper[0]->setCurrentPosition(0);
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
        stepper[0]->runForward();
      }
      break;  
    }
    case 3:
    {
      //limitStartStepper1
      stepper[1]->stopMove();
      Serial.println("limitStartStepper1 released");
      if (calibrating && !calibratedStepper[1]){
        finishCalibrateStepper(1);
      }
      break;  
    }
    case 4:
    {
      //limitEndStepper1
      stepper[1]->stopMove();
      Serial.println("limitEndStepper1 released");
      if (calibrating && !calibratedStepper[1]){
        stepper[1]->setCurrentPosition(0);
        stepper[1]->setSpeedInHz(stepper_config[0].homingSpeed);
        stepper[1]->runBackward();
      }
      break;  
    }
  }
}



// ####### END CALIBRATION #################



void loop() {
  button1.loop(); // MUST call the loop() function first
  button2.loop(); // MUST call the loop() function first
  if(button1.isPressed()){
    Serial.println("button 1 is pressed");
    onLimitSwitchPressed(1);
  }
  if(button1.isReleased()){
    Serial.println("button 1 is released");
    onLimitSwitchReleased(1);
  }
  if(button2.isPressed()){
    Serial.println("button 2 is pressed");
    onLimitSwitchPressed(2);
  }
  if(button2.isReleased()){
    Serial.println("button 2 is released");
    onLimitSwitchReleased(2);
  }
  ArduinoOTA.handle();
  artnet.parse();  // check if artnet packet has come and execute callback function

  if (millis() - oldTime > 100)
    {
        ESPUI.print(millisLabelId, String(millis()));


        oldTime = millis();
    }
}


void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    // will be called on incoming artnet data
    
    //combine two bytes into a 16 bit value
    uint16_t m1 = (data[0] * 256) + data[1];
    //webSerial.print("M1: %hi", m1);
  
    uint16_t m2 = (data[2] * 256) + data[3];
    //webSerial.print("M2: %hi", m2);

    if (calibratedStepper[0]){
      uint32_t remappedPosStepper0 = map(m1, 0, 65535, 0, maxPositionStepper[0]);
      Serial.printf(">>> moving stepper0 to: %li",remappedPosStepper0 );
      //webSerial.print(">>> moving stepper0 to: %li",remappedPosStepper0 );
      stepper[0]->moveTo(remappedPosStepper0);
    } else{
      //webSerial.println("> incoming artnet but stepper0 not calibrated");
    }
    if (calibratedStepper[1]){
      uint32_t remappedPosStepper1 = map(m1, 0, 65535, 0, maxPositionStepper[1]);
      //webSerial.print(">>> moving stepper0 to: %li",remappedPosStepper1 );
      stepper[1]->moveTo(remappedPosStepper1);
    } else{
      //webSerial.println("> incoming artnet but stepper1 not calibrated");
    }
    
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

void startHomingSteppers(bool force) {
  if (steppersHomed && !force){
    return;
  }
  homing = true;
  stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
  stepper[0]->runForward();

  //stepper[1]->setSpeedInHz(stepper_config[1].homingSpeed);
  //stepper[1]->runBackward();
  webSerial.print(">>> starting homing echt...");
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
    stepper[0]->setSpeedInHz(stepper_config[0].speed);
    stepper[1]->setSpeedInHz(stepper_config[1].speed);
  }
}
 
void sendStatus(){
  webSerial.println("");
  webSerial.printf("####### status %s ####", DNSName);
  webSerial.printf("calibrating:  %s", calibrating ? "true" : "false");
  webSerial.printf("calibratedStepper[0]: %s", calibratedStepper[0] ? "true" : "false"); 
  webSerial.printf("calibratedStepper[1]: %s", calibratedStepper[1] ? "true" : "false");
  webSerial.printf("calibrated:  %s", calibrated ? "true" : "false");
  webSerial.printf("homingDoneSinceStartup:  %s", homingDoneSinceStartup ? "true" : "false");
  webSerial.print("##########################");
}

void setSteppersAcceleration(uint32_t acceleration) {
  stepperAcceleration = acceleration;
  stepper[0]->setAcceleration(stepperAcceleration);
  stepper[1]->setAcceleration(stepperAcceleration);
}
void setSteppersSpeed(uint32_t speed) {
  stepperSpeed = speed;
  stepper[0]->setSpeedInHz(stepperSpeed);
  stepper[1]->setSpeedInHz(stepperSpeed);
}

void runForward() {
  stepper[0]->runForward();
}

void stopMotors() {
  stepper[0]->stopMove();
}