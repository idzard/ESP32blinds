#include <Arduino.h>
#include <ArtnetETH.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <MoToButtons.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <WebSerial.h>

const byte buttonPins[] = {4,35,36,39}; 
int NUMBER_BUTTONS = sizeof(buttonPins);
MoToButtons myButtons ( buttonPins, NUMBER_BUTTONS, 20, 500 ) ;

bool calibrating = false;
bool calibratedStepper[2] = {false, false};

#define stepPinStepper1 15
#define dirPinStepper1 14
#define stepPinStepper2 32
#define dirPinStepper2 33
#define DRIVER_MCPWM_PCNT 0

long m1Max = 3200;
long m2Max = 3200;

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
      homingSpeed: 2000,
      acceleration: 20000
    },
    {
      step : stepPinStepper2,
      direction : dirPinStepper2,
      speed: 7000,
      homingSpeed: 2000,
      acceleration: 20000
    }};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[2];

//ezButton limitBottomStepper0(4);  
//ezButton limitTopStepper0(35);
///ezButton limitBottomStepper1(36);
//ezButton limitTopStepper1(39);


Preferences preferences;
AsyncWebServer webserver(80);
unsigned long ota_progress_millis = 0;

const IPAddress ip(10, 0, 0, 25);
const IPAddress gateway(10, 0, 0, 1);
const IPAddress subnet_mask(255, 255, 255, 0);

ArtnetReceiver artnet;
uint16_t universe1 = 1; // 0 - 32767
uint8_t net = 0;        // 0 - 127
uint8_t subnet = 0;     // 0 - 15
uint8_t universe2 = 2;  // 0 - 15

unsigned long lastHeartbeat = 0;

void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote);
void homeSteppers();

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setup() {
  Serial.begin(115200);

  // Open Preferences with 'blinds' namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("blinds", false);
  
  ETH.begin();
  ETH.config(ip, gateway, subnet_mask);
  MDNS.begin("window1"); 
  
  //limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

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

  //if (preferences.getUInt("bottomStepper0", 0) == 0) {
    //homeSteppers();
  //}
  //else 
  //{
  //  Serial.print("Stepper0 is already homed at:");
  //  Serial.println(preferences.getUInt("bottomStepper0", 0));
  //}

  // #####  start OTA webserver ######
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is Window 1.");
  });

  ElegantOTA.begin(&webserver);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  WebSerial.begin(&webserver);
  webserver.begin();
  WebSerial.println("setup done");
}

void startCalibration(){
  calibrating = true;
  calibratedStepper[0] = false;
  calibratedStepper[1] = false;

}


void finishCalibration(uint8_t stepperId){
    // we are now back at start position. 
    // Current position will be 0 and steps traveled will be our max position.
    stepsTraveledStepper[stepperId] = stepper[stepperId]->getCurrentPosition();
    stepper[stepperId]->setCurrentPosition(0);
    maxPositionStepper[stepperId] = stepsTraveledStepper[stepperId];
    calibratedStepper[stepperId] = true;
  }



void limitSwitchPressed(int buttonId){
  switch (buttonId) {
    case 0:
    {
      //limitStartStepper0
      stepper[0]->stopMove();
      if (calibrating){
        finishCalibration();
      }  
    }
    case 1:
    {
      //limitEndStepper0
      stepper[0]->stopMove();
      if (calibrating){
        stepper[0]->runBackward();
      }  
    }
    case 2:
    {
      //limitBottomStepper1
      stepper[1]->stopMove();
    }
    case 3:
    {
      //limitTopStepper1
      stepper[1]->stopMove();
    }
}


void loop() {
  myButtons.processButtons();

  for (int i = 0; i < (NUMBER_BUTTONS); i = i + 1) {

      if (myButtons.pressed(i)) {
      WebSerial.print("Button pressed: ");
      WebSerial.println(i);
      Serial.print("Button Pressed ");
      Serial.println(i);
      }
  }

  
  
  /*limitSwitch.loop();
  
  if(limitSwitch.isPressed())
  {
    stepper[0]->forceStop();
    stepper[1]->forceStop();
    Serial.println("Stepper0 is homed at:");
    Serial.println(stepper[0]->getCurrentPosition());
    // Store the position and close the Preferences
    preferences.putUInt("bottomStepper0", stepper[0]->getCurrentPosition());
    preferences.end();
  }
  if(limitSwitch.isReleased())
    Serial.println("The button is released");
  */
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
    Serial.print("M1: ");
    Serial.print(m1);
    Serial.println("");

    uint16_t m2 = (data[2] * 256) + data[3];
    Serial.print("M2: ");
    Serial.print(m2);
    Serial.println("");

    
    Serial.println("Art-Net data received:");
    Serial.print(remote.ip);
    Serial.print(":");
   
    for (size_t i = 0; i < 4; ++i) {
        Serial.print(data[i]);
        Serial.print(",");
    }
    Serial.println();

    uint16_t m1Remapped = map(m1, 0, 65535, 0, m1Max);
    uint16_t m2Remapped = map(m2, 0, 65535, 0, m2Max);
    Serial.print("moving stepper0 to: ");
    Serial.print(m1Remapped);
    Serial.println();
    Serial.print("moving stepper1 to: ");
    Serial.print(m2Remapped);
    Serial.println();
    stepper[0]->moveTo(m1Remapped);
    stepper[1]->moveTo(m2Remapped);
}


void homeSteppers() {
  stepper[0]->setCurrentPosition(0);
  stepper[0]->setSpeedInHz(stepper_config[0].homingSpeed);
  stepper[0]->runForward();

  stepper[1]->setCurrentPosition(0);
  stepper[1]->setSpeedInHz(stepper_config[1].homingSpeed);
  stepper[1]->runForward();
}