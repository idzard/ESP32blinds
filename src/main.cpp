#include <Arduino.h>
#include <ArtnetETH.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ezButton.h>

//#include <MoToButtons.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <MycilaWebSerial.h>
#include "configLoader.h"


const byte buttonPins[] = {4,35}; 
int NUMBER_BUTTONS = sizeof(buttonPins);

//MoToButtons myButtons ( buttonPins, NUMBER_BUTTONS, 50, 500 ) ;
ezButton button1(4);
ezButton button2(35);


#define stepPinStepper1 15
#define dirPinStepper1 14
#define stepPinStepper2 32
#define dirPinStepper2 33
#define DRIVER_MCPWM_PCNT 0

long m1Max = 3200;
long m2Max = 3200;

bool homing = false;
bool homedStepper[2] = {false, false};
bool steppersHomed = false;
bool calibrating = false;
bool calibrated = false;
bool calibratedStepper[2] = {false, false};

uint32_t stepsTraveledStepper[2];
uint32_t minPositionStepper[2] = {0,0};
uint32_t maxPositionStepper[2];

struct stepper_config_s {
  uint8_t step;
  uint8_t direction;
  uint32_t speed;
  uint32_t homingSpeed;
  uint32_t acceleration;
};

const struct stepper_config_s stepper_config[2] = {
    {
      step : stepPinStepper1,
      direction : dirPinStepper1,
      speed: 7000,
      homingSpeed: 1000,
      acceleration: 20000
    },
    {
      step : stepPinStepper2,
      direction : dirPinStepper2,
      speed: 7000,
      homingSpeed: 1000,
      acceleration: 20000
    }};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[2];

Preferences preferences;
AsyncWebServer webserver(80);


//const IPAddress ip(10, 0, 0, 25);
///const IPAddress gateway(10, 0, 0, 1);
//const IPAddress subnet_mask(255, 255, 255, 0);

ArtnetReceiver artnet;
uint16_t universe1 = 1; // 0 - 32767
uint8_t net = 0;        // 0 - 127
uint8_t subnet = 0;     // 0 - 15
uint8_t universe2 = 2;  // 0 - 15

unsigned long lastHeartbeat = 0;

//forward declarations
void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote);
void startHomingSteppers();
void startCalibration();
void finishHomingStepper(uint8_t stepperId);



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
  
  //check if we can load calibration from disk
  int savedStepper0 = preferences.getUInt("maxPositionStepper0", 0);
  if (savedStepper0 != 0){
    maxPositionStepper[0] = savedStepper0;
    calibratedStepper[0] = true;
    Serial.print("stepper0 calibration found:");
    Serial.println(maxPositionStepper[0]);
  }
  int savedStepper1 = preferences.getUInt("maxPositionStepper1", 0);
  if (savedStepper1 != 0){
    maxPositionStepper[1] = savedStepper1;
    calibratedStepper[1] = true;
    Serial.print("stepper1 calibration found:");
    Serial.println(maxPositionStepper[1]);
  }
  if (calibratedStepper[0] == true && calibratedStepper[1] == true){
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
        s->setDirectionPin(config->direction);
        s->setSpeedInHz(config->speed);
        s->setAcceleration(config->acceleration);
      }
    }
    stepper[i] = s;
  }
  
  artnet.begin();
  artnet.subscribeArtDmxUniverse(net, subnet, universe1, onArtnetReceive);


  // #####  start OTA webserver ######
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is Window 1.");
  });

  ElegantOTA.begin(&webserver);    // Start ElegantOTA

 
  
  /*/* Attach webserial Message Callback 
  WebSerial.onMessage([&](uint8_t *data, size_t len) {

    WebSerial.println("Received Data...");
    String message = "";
    for(size_t i=0; i < len; i++){
      message += char(data[i]);
    }
    WebSerial.println(message);
    if (message == "calibrate"){
      startCalibration();
    }
  });*/
 

  WebSerial.begin(&webserver);
  webserver.begin();
  WebSerial.println("setup done");

  //startHomingSteppers();
}


// ####### START CALIBRATION #################

void startCalibration(){
  calibrating = true;
  calibratedStepper[0] = false;
  calibratedStepper[1] = false;

  stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
  stepper[0]->runForward();
  
  stepper[1]->setSpeedInHz(stepper_config[1].homingSpeed);
  stepper[1]->runForward();
}


void finishCalibrateStepper(uint8_t stepperId){
  // we are now back at start position. 
  // Current position will be 0 and steps traveled will be our max position.
  stepsTraveledStepper[stepperId] = stepper[stepperId]->getCurrentPosition();
  stepper[stepperId]->setCurrentPosition(0);
  maxPositionStepper[stepperId] = stepsTraveledStepper[stepperId];
  calibratedStepper[stepperId] = true;
  
  if (calibratedStepper[0] == true && calibratedStepper[1] == true){
    //all calibration done
    
    // Store the position and close the Preferences
    preferences.putUInt("maxPositionStepper0",maxPositionStepper[0]);
    preferences.putUInt("maxPositionStepper1",maxPositionStepper[1]);

    preferences.end();
    
    calibrating = false;

  }
}



void onLimitSwitchPressed(int buttonId){
  Serial.println(buttonId);
  switch (buttonId) {
    case 1:
    {
      //limitStartStepper0
      stepper[0]->stopMove();
      Serial.println("limitStartStepper0 pressed");
      if (homing){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runForward();
        Serial.println("slow homing forward");
      }
      
      
      if (calibrating && !calibratedStepper[0]){
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed/10);
        stepper[0]->runForward();
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
        stepper[0]->runBackward();
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
      if (calibrating && !calibratedStepper[0]){
        stepper[0]->setCurrentPosition(0);
        stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
        stepper[0]->runBackward();
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
    onLimitSwitchPressed(3);
  }
  if(button2.isReleased()){
    Serial.println("button 2 is released");
    onLimitSwitchReleased(3);
  }
  artnet.parse();  // check if artnet packet has come and execute callback function
  ElegantOTA.loop();

  if (millis() > lastHeartbeat + 1000){
      Serial.print(".");
      lastHeartbeat = millis();
  }
}


void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    // will be called on incoming artnet data
    
    Serial.println("");
    Serial.print(data[0]);
    Serial.print("-");
    Serial.print(data[1]);
    Serial.println("");
    
    //combine two bytes into a 16 bit value
    uint16_t m1 = (data[0] * 256) + data[1];
    WebSerial.print("M1: ");
    WebSerial.println(m1);
    

    uint16_t m2 = (data[2] * 256) + data[3];
    WebSerial.print("M2: ");
    WebSerial.println(m2);


    
    Serial.println("Art-Net data received:");
    Serial.print(remote.ip);
    Serial.print(":");
   
    for (size_t i = 0; i < 4; ++i) {
        Serial.print(data[i]);
        Serial.print(",");
    }
    Serial.println();

    //if (calibratedStepper[0]){
      uint16_t remappedPosStepper0 = map(m1, 0, 65535, 0, 3000);
      WebSerial.print("moving stepper0 to: ");
      WebSerial.println(remappedPosStepper0);
      stepper[0]->moveTo(remappedPosStepper0);
    //} else{
    //  Serial.println("incoming artnet but stepper0 not calibrated");
    //}
    //if (calibratedStepper[1]){
      uint16_t remappedPosStepper1 = map(m1, 0, 65535, 0, 3000);
      WebSerial.print("moving stepper1 to: ");
      WebSerial.println(remappedPosStepper1);
      stepper[1]->moveTo(remappedPosStepper1);
    //} else{
    //  Serial.println("incoming artnet but stepper1 not calibrated");
    //}
    
}


void startHomingSteppers() {
  if (steppersHomed){
    return;
  }
  homing = true;
  stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
  stepper[0]->runBackward();

  stepper[1]->setSpeedInHz(stepper_config[1].homingSpeed);
  stepper[1]->runBackward();
  Serial.println("start homing");
}
  
void finishHomingStepper(uint8_t stepperId){
  stepper[stepperId]->stopMove();
  stepper[stepperId]->setCurrentPosition(0);
  homedStepper[stepperId] = true;
  Serial.print("done homing stepper:");
  Serial.println(stepperId);
  if (homedStepper[0] ==true && homedStepper[1] == true){
    steppersHomed = true;
    homing = false;
  }
}
 
